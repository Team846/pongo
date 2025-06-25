#include "frc846/control/MotorMonkey.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>

#include <iostream>
#include <string_view>

#include "frc846/control/hardware/SparkMXFX_interm.h"
#include "frc846/control/hardware/TalonFX_interm.h"
#include "frc846/control/hardware/simulation/MCSimulator.h"
#include "frc846/control/hardware/simulation/SIMLEVEL.h"
#include "frc846/math/collection.h"

namespace frc846::control {

#define CHECK_SLOT_ID()                                                      \
  if (controller_registry[slot_id] == nullptr)                               \
    throw std::runtime_error(                                                \
        "Invalid MotorMonkey slot ID: " + std::to_string(slot_id) + " in " + \
        "[" + __func__ + "]");

#define LOG_IF_ERROR(action_name)                                          \
  {                                                                        \
    hardware::ControllerErrorCodes err =                                   \
        controller_registry[slot_id]->GetLastErrorCode();                  \
    if (err != hardware::ControllerErrorCodes::kAllOK) {                   \
      loggable_.Error("Error [{}] completing action [{}] for slot ID {}.", \
          parseError(err), action_name, slot_id);                          \
    }                                                                      \
  }

#define NUM_RETRIES 5
#define INITIAL_RETRY_DELAY_MS 10

#define SMART_RETRY(action, action_name)                                    \
  for (int i = 0; i < NUM_RETRIES; i++) {                                   \
    action;                                                                 \
    hardware::ControllerErrorCodes err =                                    \
        controller_registry[slot_id]->GetLastErrorCode();                   \
    if (err == hardware::ControllerErrorCodes::kAllOK)                      \
      break;                                                                \
    else if (i == NUM_RETRIES - 1) {                                        \
      loggable_.Error("Failed [{}] for slot ID {}.", action_name, slot_id); \
    } else {                                                                \
      loggable_.Warn(                                                       \
          "Error [{}] while attempting [{}] for slot ID {}. Retrying...",   \
          parseError(err), action_name, slot_id);                           \
      std::this_thread::sleep_for(                                          \
          std::chrono::milliseconds(INITIAL_RETRY_DELAY_MS * (1 << i)));    \
    }                                                                       \
  }

frc846::base::Loggable MotorMonkey::loggable_{"MotorMonkey"};

size_t MotorMonkey::slot_counter_{0};
std::map<size_t, frc846::control::base::MotorMonkeyType>
    MotorMonkey::slot_id_to_type_{};
std::map<size_t, bool> MotorMonkey::slot_id_to_sim_{};

frc846::control::hardware::IntermediateController*
    MotorMonkey::controller_registry[CONTROLLER_REGISTRY_SIZE]{};

frc846::control::base::MotorGains
    MotorMonkey::gains_registry[CONTROLLER_REGISTRY_SIZE]{};

units::newton_meter_t MotorMonkey::load_registry[CONTROLLER_REGISTRY_SIZE]{};

frc846::wpilib::unit_ohm
    MotorMonkey::circuit_resistance_registry[CONTROLLER_REGISTRY_SIZE]{};

units::volt_t MotorMonkey::battery_voltage{0_V};

units::volt_t MotorMonkey::last_disabled_voltage{0_V};

frc846::math::DoubleSyncBuffer MotorMonkey::sync_buffer{100U, 35};

units::ampere_t MotorMonkey::max_draw_{0.0_A};

std::queue<MotorMonkey::MotorMessage> MotorMonkey::control_messages{};

int MotorMonkey::num_loops_last_brown = 4000;

void MotorMonkey::Setup() {
  loggable_.RegisterPreference("voltage_min", 8.0_V);
  loggable_.RegisterPreference("recal_voltage_thresh", 10.5_V);
  loggable_.RegisterPreference("default_max_draw", 150.0_A);
  loggable_.RegisterPreference("min_max_draw", 60_A);
  loggable_.RegisterPreference("max_max_draw", 250_A);
  loggable_.RegisterPreference("battery_cc", 700_A);
  loggable_.RegisterPreference("brownout_perm_loops", 500);

  max_draw_ = loggable_.GetPreferenceValue_unit_type<units::ampere_t>(
      "default_max_draw");
}

void MotorMonkey::RecalculateMaxDraw() {
  if (!sync_buffer.IsValid()) return;

  if (frc::RobotController::IsBrownedOut()) num_loops_last_brown = 0;
  if (num_loops_last_brown <
      loggable_.GetPreferenceValue_int("brownout_perm_loops")) {
    max_draw_ =
        loggable_.GetPreferenceValue_unit_type<units::ampere_t>("min_max_draw");
    return;
  }

  sync_buffer.Sync();
  // loggable_.Graph("sync_diff_", sync_buffer.GetSyncDiff());

  auto sy_trough = sync_buffer.GetTrough();
  units::ampere_t cc_current{sy_trough.first};

  // loggable_.Graph("cc_current", cc_current);

  units::ampere_t current_draw =
      loggable_.GetPreferenceValue_unit_type<units::ampere_t>("battery_cc") -
      cc_current;
  units::volt_t voltage{sy_trough.second};

  units::volt_t voltage_min =
      loggable_.GetPreferenceValue_unit_type<units::volt_t>("voltage_min");
  units::volt_t recal_voltage_thresh =
      loggable_.GetPreferenceValue_unit_type<units::volt_t>(
          "recal_voltage_thresh");

  if (voltage > recal_voltage_thresh) return;

  units::ampere_t temp_max_draw_ = current_draw *
                                   (last_disabled_voltage - voltage_min) /
                                   (last_disabled_voltage - voltage);

  if (temp_max_draw_ <
      loggable_.GetPreferenceValue_unit_type<units::ampere_t>("min_max_draw")) {
    return;
  } else if (temp_max_draw_ >
             loggable_.GetPreferenceValue_unit_type<units::ampere_t>(
                 "max_max_draw")) {
    return;
  } else {
    max_draw_ = temp_max_draw_;
  }
}

void MotorMonkey::Tick(bool disabled) {
  battery_voltage = frc::RobotController::GetBatteryVoltage();
  // loggable_.Graph("battery_voltage", battery_voltage);
  loggable_.Graph("last_disabled_voltage", last_disabled_voltage);

  if (disabled) {
    last_disabled_voltage = battery_voltage;
    if (!frc::RobotBase::IsSimulation()) {
      WriteMessages(400_A);  // TODO: fix
      return;
    }
  }

  units::ampere_t total_pred_draw = WriteMessages(max_draw_);

  loggable_.Graph("total_pred_draw", total_pred_draw);

  units::ampere_t cc_current =
      loggable_.GetPreferenceValue_unit_type<units::ampere_t>("battery_cc") -
      total_pred_draw;
  sync_buffer.Add(cc_current.to<double>(), battery_voltage.to<double>());

  RecalculateMaxDraw();

  loggable_.Graph("max_draw", max_draw_);

  for (size_t i = 0; i < CONTROLLER_REGISTRY_SIZE; i++) {
    if (controller_registry[i] != nullptr) {
      if (slot_id_to_sim_[i]) {
        simulation::MCSimulator* sim =
            dynamic_cast<simulation::MCSimulator*>(controller_registry[i]);
        sim->SetBatteryVoltage(battery_voltage);
        sim->SetLoad(load_registry[i]);
        if (disabled) {
          sim->WriteDC(0.0);
          // std::cout << "MotorMonkey: Disabled, setting DC to 0.0" <<
          // std::endl;
        }

        // std::cout << "MotorMonkey: Tick for slot ID " << i
        //           << ", sim: " << (sim != nullptr) << std::endl;
        sim->Tick();
        // std::cout << "--" << std::endl;
      } else {
        controller_registry[i]->Tick();
      }
    }
  }
}

void MotorMonkey::SetNeutralMode(size_t slot_id, bool brake_mode) {
  CHECK_SLOT_ID();
  controller_registry[slot_id]->SetNeutralMode(brake_mode);
  LOG_IF_ERROR("SetNeutralMode");
}

units::ampere_t MotorMonkey::WriteMessages(units::ampere_t max_draw) {
  units::ampere_t total_current = 0.0_A;
  std::queue<MotorMessage> temp_messages{control_messages};

  double scale_factor = 1.0;

  while (!temp_messages.empty()) {
    MotorMessage& msg = temp_messages.front();

    frc846::control::base::MotorMonkeyType motor_type =
        slot_id_to_type_[msg.slot_id];

    units::radians_per_second_t velocity =
        controller_registry[msg.slot_id]->GetVelocity();

    double duty_cycle = 0.0;

    switch (msg.type) {
    case MotorMessage::Type::DC:
      duty_cycle = std::get<double>(msg.value);
      break;
    case MotorMessage::Type::Position: {
      // Onboard control should only be used with low-current draw systems
      duty_cycle = 0.0;
      break;
    }
    case MotorMessage::Type::Velocity: {
      // Onboard control should only be used with low-current draw systems
      duty_cycle = 0.0;
      break;
    }
    default:
      throw std::runtime_error(
          "Unsupported MotorMessage type in MotorMonkey WriteMessages");
    }
    units::ampere_t pred_draw =
        frc846::control::calculators::CurrentTorqueCalculator::
            predict_current_draw(duty_cycle, velocity, battery_voltage,
                circuit_resistance_registry[msg.slot_id], motor_type);

    if (velocity > 0_rad_per_s && pred_draw < 0_A) {
      (void)pred_draw;  // Regen braking mode
    } else if (velocity < 0_rad_per_s && pred_draw > 0_A) {
      (void)pred_draw;  // Regen braking mode
    } else {
      total_current += units::math::abs(pred_draw);
    }
    temp_messages.pop();
  }

  if (total_current > 1000_A) { total_current = 1000_A; }

  if (total_current > max_draw && max_draw > 0.0_A) {
    scale_factor = max_draw / total_current;
    loggable_.Warn("Brownout predicted. Rescaling outputs by {} to resolve.",
        scale_factor);
  }

  loggable_.Graph("pred_current_draw", total_current.to<double>());
  loggable_.Graph("current_scale_factor", scale_factor);

  // loggable_.Graph("num_control_messages", (int)control_messages.size());

  while (!control_messages.empty()) {
    const auto& msg = control_messages.front();
    auto* controller = controller_registry[msg.slot_id];

    units::radians_per_second_t velocity =
        controller_registry[msg.slot_id]->GetVelocity();

    frc846::control::base::MotorMonkeyType motor_type =
        slot_id_to_type_[msg.slot_id];

    size_t slot_id = msg.slot_id;

    switch (msg.type) {
    case MotorMessage::Type::DC: {
      double scaled_duty_cycle = std::get<double>(msg.value);
      frc846::control::calculators::CurrentTorqueCalculator::scale_current_draw(
          scale_factor, std::get<double>(msg.value), velocity, battery_voltage,
          motor_type);
      if (!controller->IsDuplicateControlMessage(scaled_duty_cycle) ||
          controller->GetLastErrorCode() !=
              frc846::control::hardware::ControllerErrorCodes::kAllOK) {
        controller->WriteDC(scaled_duty_cycle);
        LOG_IF_ERROR("WriteDC");
      }
      break;
    }
    case MotorMessage::Type::Position: {
      auto pos = std::get<units::radian_t>(msg.value);
      if (!controller->IsDuplicateControlMessage(pos) ||
          controller->GetLastErrorCode() !=
              frc846::control::hardware::ControllerErrorCodes::kAllOK) {
        controller->WritePosition(pos);
        LOG_IF_ERROR("WritePosition");
      }
      break;
    }
    case MotorMessage::Type::Velocity: {
      auto vel = std::get<units::radians_per_second_t>(msg.value);
      if (!controller->IsDuplicateControlMessage(vel) ||
          controller->GetLastErrorCode() !=
              frc846::control::hardware::ControllerErrorCodes::kAllOK) {
        controller->WriteVelocity(vel);
        LOG_IF_ERROR("WriteVelocity");
      }
      break;
    }
    }

    control_messages.pop();
  }
  return total_current;
}

bool MotorMonkey::VerifyConnected() {
  for (size_t i = 0; i < CONTROLLER_REGISTRY_SIZE; i++) {
    if (controller_registry[i] != nullptr) {
      if (!controller_registry[i]->VerifyConnected()) { return false; }
    }
  }
  return true;
}

size_t MotorMonkey::ConstructController(
    frc846::control::base::MotorMonkeyType type,
    frc846::control::config::MotorConstructionParameters params) {
  slot_counter_++;

  size_t slot_id = slot_counter_;
  slot_id_to_type_[slot_id] = type;
  slot_id_to_sim_[slot_id] = false;
  circuit_resistance_registry[slot_id] = params.circuit_resistance;

  frc846::control::hardware::IntermediateController* this_controller = nullptr;

  if (frc::RobotBase::IsSimulation() &&
      MOTOR_SIM_LEVEL == MOTOR_SIM_LEVEL_SIM_PHYSICS) {
    std::cout << "Constructing simulation controller" << std::endl;
    loggable_.Log(
        "Constructing physics simulation controller for slot ID {}.", slot_id);
    slot_id_to_sim_[slot_id] = true;
    this_controller = controller_registry[slot_id] =
        new frc846::control::simulation::MCSimulator{
            frc846::control::base::MotorSpecificationPresets::get(type),
            params.circuit_resistance, params.rotational_inertia,
            params.friction};
  } else if (frc846::control::base::MotorMonkeyTypeHelper::is_talon_fx(type)) {
    this_controller = controller_registry[slot_id] =
        new frc846::control::hardware::TalonFX_interm{
            params.can_id, params.bus, params.max_wait_time};

  } else if (frc846::control::base::MotorMonkeyTypeHelper::is_spark_max(type)) {
    this_controller = controller_registry[slot_id] =
        new frc846::control::hardware::SparkMAX_interm{
            params.can_id, params.max_wait_time};
  } else if (frc846::control::base::MotorMonkeyTypeHelper::is_spark_flex(
                 type)) {
    this_controller = controller_registry[slot_id] =
        new frc846::control::hardware::SparkFLEX_interm{
            params.can_id, params.max_wait_time};
  } else {
    throw std::runtime_error("Invalid MotorMonkeyType [" +
                             std::to_string((int)type) +
                             "]: not constructing controller");
  }

  if (this_controller == nullptr) { return slot_id; }

  SMART_RETRY(this_controller->SetInverted(params.inverted), "SetInverted");
  LOG_IF_ERROR("SetInverted");

  SMART_RETRY(
      this_controller->SetNeutralMode(params.brake_mode), "SetNeutralMode");
  LOG_IF_ERROR("SetNeutralMode");

  SMART_RETRY(this_controller->SetCurrentLimit(params.motor_current_limit),
      "SetCurrentLimit");
  LOG_IF_ERROR("SetCurrentLimit");

  SMART_RETRY(
      this_controller->SetVoltageCompensation(params.voltage_compensation),
      "SetSoftLimits");
  LOG_IF_ERROR("SetVoltageCompensation")

  return slot_id;
}

void MotorMonkey::EnableStatusFrames(size_t slot_id,
    std::vector<frc846::control::config::StatusFrame> frames,
    units::millisecond_t faults_ms, units::millisecond_t velocity_ms,
    units::millisecond_t encoder_position_ms,
    units::millisecond_t analog_position_ms) {
  CHECK_SLOT_ID();

  SMART_RETRY(
      controller_registry[slot_id]->EnableStatusFrames(frames, faults_ms,
          velocity_ms, encoder_position_ms, analog_position_ms),
      "EnableStatusFrames");
  LOG_IF_ERROR("EnableStatusFrames");
}

void MotorMonkey::OverrideStatusFramePeriod(size_t slot_id,
    frc846::control::config::StatusFrame frame, units::millisecond_t period) {
  CHECK_SLOT_ID();
  SMART_RETRY(
      controller_registry[slot_id]->OverrideStatusFramePeriod(frame, period),
      "OverrideStatusFramePeriod");
  LOG_IF_ERROR("OverrideStatusFramePeriod");
}

units::volt_t MotorMonkey::GetBatteryVoltage() { return battery_voltage; }

void MotorMonkey::SetLoad(size_t slot_id, units::newton_meter_t load) {
  CHECK_SLOT_ID();

  load_registry[slot_id] = load;
}

void MotorMonkey::SetGains(
    size_t slot_id, frc846::control::base::MotorGains gains) {
  CHECK_SLOT_ID();

  if (frc846::math::DEquals(gains_registry[slot_id].kP, gains.kP) &&
      frc846::math::DEquals(gains_registry[slot_id].kI, gains.kI) &&
      frc846::math::DEquals(gains_registry[slot_id].kD, gains.kD) &&
      frc846::math::DEquals(gains_registry[slot_id].kFF, gains.kFF)) {
    return;
  } else {
    gains_registry[slot_id] = gains;
  }

  SMART_RETRY(controller_registry[slot_id]->SetGains(gains), "SetGains");
  LOG_IF_ERROR("SetGains");
}

void MotorMonkey::WriteDC(size_t slot_id, double duty_cycle) {
  CHECK_SLOT_ID();

  control_messages.push({slot_id, MotorMessage::Type::DC, duty_cycle});
}

void MotorMonkey::WriteVelocity(
    size_t slot_id, units::radians_per_second_t velocity) {
  CHECK_SLOT_ID();

  control_messages.push({slot_id, MotorMessage::Type::Velocity, velocity});
}

void MotorMonkey::WritePosition(size_t slot_id, units::radian_t position) {
  CHECK_SLOT_ID();

  control_messages.push({slot_id, MotorMessage::Type::Position, position});
}

void MotorMonkey::ZeroEncoder(size_t slot_id, units::radian_t position) {
  CHECK_SLOT_ID();

  SMART_RETRY(
      controller_registry[slot_id]->ZeroEncoder(position), "ZeroEncoder");
  LOG_IF_ERROR("ZeroEncoder");
}

units::radians_per_second_t MotorMonkey::GetVelocity(size_t slot_id) {
  CHECK_SLOT_ID();

  return controller_registry[slot_id]->GetVelocity();
}

units::radian_t MotorMonkey::GetPosition(size_t slot_id) {
  CHECK_SLOT_ID();

  return controller_registry[slot_id]->GetPosition();
}

units::ampere_t MotorMonkey::GetCurrent(size_t slot_id) {
  CHECK_SLOT_ID();

  return controller_registry[slot_id]->GetCurrent();
}

void MotorMonkey::ConfigForwardLimitSwitch(size_t slot_id, bool stop_motor,
    frc846::control::base::LimitSwitchDefaultState type) {
  CHECK_SLOT_ID();

  SMART_RETRY(
      controller_registry[slot_id]->ConfigForwardLimitSwitch(stop_motor, type),
      "ConfigForwardLimitSwitch");
  LOG_IF_ERROR("ConfigForwardLimitSwitch");
}

void MotorMonkey::ConfigReverseLimitSwitch(size_t slot_id, bool stop_motor,
    frc846::control::base::LimitSwitchDefaultState type) {
  CHECK_SLOT_ID();

  SMART_RETRY(
      controller_registry[slot_id]->ConfigReverseLimitSwitch(stop_motor, type),
      "ConfigReverseLimitSwitch");
  LOG_IF_ERROR("ConfigReverseLimitSwitch");
}

bool MotorMonkey::GetForwardLimitSwitchState(size_t slot_id) {
  CHECK_SLOT_ID();

  return controller_registry[slot_id]->GetForwardLimitSwitchState();
}

bool MotorMonkey::GetReverseLimitSwitchState(size_t slot_id) {
  CHECK_SLOT_ID();

  return controller_registry[slot_id]->GetReverseLimitSwitchState();
}

units::turn_t MotorMonkey::GetAbsoluteEncoderPosition(size_t slot_id) {
  CHECK_SLOT_ID();

  return controller_registry[slot_id]->GetAbsoluteEncoderPosition();
}

void MotorMonkey::SetSoftLimits(size_t slot_id, units::radian_t forward_limit,
    units::radian_t reverse_limit) {
  CHECK_SLOT_ID();

  SMART_RETRY(
      controller_registry[slot_id]->SetSoftLimits(forward_limit, reverse_limit),
      "SetSoftLimits");
  LOG_IF_ERROR("SetSoftLimits");
}

std::string_view MotorMonkey::parseError(
    frc846::control::hardware::ControllerErrorCodes err) {
  switch (err) {
  case frc846::control::hardware::ControllerErrorCodes::kAllOK: return "All OK";
  case frc846::control::hardware::ControllerErrorCodes::kInvalidCANID:
    return "Invalid CAN ID";
  case frc846::control::hardware::ControllerErrorCodes::kVersionMismatch:
    return "Version Mismatch";
  case frc846::control::hardware::ControllerErrorCodes::kHALError:
    return "HAL Error";
  case frc846::control::hardware::ControllerErrorCodes::kFollowingError:
    return "Following Error";
  case frc846::control::hardware::ControllerErrorCodes::kCANDisconnected:
    return "CAN Disconnected";
  case frc846::control::hardware::ControllerErrorCodes::kCANMessageStale:
    return "CAN Message Stale";
  case frc846::control::hardware::ControllerErrorCodes::kDeviceDisconnected:
    return "Device Disconnected";
  case frc846::control::hardware::ControllerErrorCodes::kConfigFailed:
    return "Config Failed";
  case frc846::control::hardware::ControllerErrorCodes::kTimeout:
    return "Timeout";
  case frc846::control::hardware::ControllerErrorCodes::kWarning:
    return "Unknown Warning";
  case frc846::control::hardware::ControllerErrorCodes::kError:
    return "Unknown Error";
  default: return "Unknown";
  }
}

}  // namespace frc846::control