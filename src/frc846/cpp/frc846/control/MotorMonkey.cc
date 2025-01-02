#include "frc846/control/MotorMonkey.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>

#include "frc846/control/hardware/TalonFX_interm.h"
#include "frc846/control/hardware/SparkMXFX_interm.h"

#include "frc846/math/collection.h"

#include <string>

// TODO: Add dynamic can/power management

namespace frc846::control {

#define CHECK_SLOT_ID()                        \
  if (slot_id >= CONTROLLER_REGISTRY_SIZE ||   \
      controller_registry[slot_id] == nullptr) \
  throw std::runtime_error("Invalid MotorMonkey slot ID")

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

#define SMART_RETRY(action, action_name)                                  \
  for (int i = 0; i < NUM_RETRIES; i++) {                                 \
    action;                                                               \
    hardware::ControllerErrorCodes err =                                  \
        controller_registry[slot_id]->GetLastErrorCode();                 \
    if (err == hardware::ControllerErrorCodes::kAllOK)                    \
      break;                                                              \
    else {                                                                \
      loggable_.Warn(                                                     \
          "Error [{}] while attempting [{}] for slot ID {}. Retrying...", \
          parseError(err), action_name, slot_id);                         \
      std::this_thread::sleep_for(                                        \
          std::chrono::milliseconds(INITIAL_RETRY_DELAY_MS * (1 << i)));  \
    }                                                                     \
  }

frc846::base::Loggable MotorMonkey::loggable_{"MotorMonkey"};

size_t MotorMonkey::slot_counter_{0};
std::map<size_t, frc846::control::base::MotorMonkeyType>
    MotorMonkey::slot_id_to_type_{};

frc846::control::hardware::IntermediateController*
    MotorMonkey::controller_registry[CONTROLLER_REGISTRY_SIZE]{};

frc846::control::base::MotorGains
    MotorMonkey::gains_registry[CONTROLLER_REGISTRY_SIZE]{};

units::newton_meter_t MotorMonkey::load_registry[CONTROLLER_REGISTRY_SIZE]{};

frc846::wpilib::unit_ohm
    MotorMonkey::circuit_resistance_registry[CONTROLLER_REGISTRY_SIZE]{};

units::volt_t MotorMonkey::battery_voltage{0_V};

std::queue<MotorMonkey::MotorMessage> MotorMonkey::control_messages{};

void MotorMonkey::Tick() {
  battery_voltage = frc::RobotController::GetBatteryVoltage();
  WriteMessages();

  for (size_t i = 0; i < CONTROLLER_REGISTRY_SIZE; i++) {
    if (controller_registry[i] != nullptr) { controller_registry[i]->Tick(); }
  }
}

void MotorMonkey::WriteMessages() {
  frc846::wpilib::unit_ohm total_resistance{0.0};
  units::ampere_t total_current = 0.0_A;
  std::queue<MotorMessage> temp_messages = control_messages;
  double scale_factor = 1.0;

  while (!temp_messages.empty()) {
    const auto& msg = temp_messages.front();
    auto type = slot_id_to_type_[msg.slot_id];
    auto velocity = controller_registry[msg.slot_id]->GetVelocity();
    double load = load_registry[msg.slot_id].to<double>();
    auto gains = gains_registry[msg.slot_id];
    total_resistance += circuit_resistance_registry[msg.slot_id];
    double duty_cycle = 0.0;

    switch (msg.type) {
    case MotorMessage::Type::DC:
      duty_cycle = std::get<double>(msg.value);
      break;
    case MotorMessage::Type::Position: {
      duty_cycle = std::clamp(
          gains.calculate((controller_registry[msg.slot_id]->GetPosition() -
                              std::get<units::radian_t>(msg.value))
                              .to<double>(),
              0.0, velocity.to<double>(), load),
          -1.0, 1.0);
      break;
    }
    case MotorMessage::Type::Velocity: {
      duty_cycle = std::clamp(
          gains.calculate(
              (velocity - std::get<units::radians_per_second_t>(msg.value))
                  .to<double>(),
              0.0, 0.0, load),
          -1.0, 1.0);
      break;
    }
    }
    total_current += frc846::control::calculators::CurrentTorqueCalculator::
        predict_current_draw(duty_cycle, velocity, battery_voltage,
            circuit_resistance_registry[msg.slot_id], type);
    temp_messages.pop();
  }

  units::volt_t voltage_drop = total_current * total_resistance;
  units::volt_t predicted_voltage = battery_voltage - voltage_drop;

  if (voltage_drop > 0.0_V && predicted_voltage < 7.0_V) {
    scale_factor = (battery_voltage - 7_V) / voltage_drop;
  }

  while (!control_messages.empty()) {
    const auto& msg = control_messages.front();
    auto* controller = controller_registry[msg.slot_id];
    [&, slot_id = msg.slot_id]() {
      switch (msg.type) {
      case MotorMessage::Type::DC: {
        double dc = std::get<double>(msg.value) * scale_factor;
        if (!controller->IsDuplicateControlMessage(dc)) {
          controller->WriteDC(dc);
          LOG_IF_ERROR("WriteDC");
        }
        break;
      }
      case MotorMessage::Type::Position: {
        auto pos = std::get<units::radian_t>(msg.value);
        if (!controller->IsDuplicateControlMessage(pos)) {
          controller->WritePosition(pos);
          LOG_IF_ERROR("WritePosition");
        }
        break;
      }
      case MotorMessage::Type::Velocity: {
        auto vel = std::get<units::radians_per_second_t>(msg.value);
        if (!controller->IsDuplicateControlMessage(vel)) {
          controller->WriteVelocity(vel);
          LOG_IF_ERROR("WriteVelocity");
        }
        break;
      }
      }
    }();
    control_messages.pop();
  }
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
  circuit_resistance_registry[slot_id] = params.circuit_resistance;

  frc846::control::hardware::IntermediateController* this_controller = nullptr;

  if (frc::RobotBase::IsSimulation()) {
    // TODO: implement sim controllers
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
  }

  if (this_controller == nullptr) return slot_id;

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

units::volt_t MotorMonkey::GetBatteryVoltage() { return battery_voltage; }

void MotorMonkey::SetLoad(size_t slot_id, units::newton_meter_t load) {
  CHECK_SLOT_ID();

  load_registry[slot_id] = load;

  if (controller_registry[slot_id] != nullptr) {
    // TODO: If is MCSim, cast then set load
    // controller_registry[slot_id]->SetLoad(load);
  }
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

void MotorMonkey::SetSoftLimits(size_t slot_id, units::radian_t forward_limit,
    units::radian_t reverse_limit) {
  CHECK_SLOT_ID();

  SMART_RETRY(
      controller_registry[slot_id]->SetSoftLimits(forward_limit, reverse_limit),
      "SetSoftLimits");
  LOG_IF_ERROR("SetSoftLimits");
}

std::string MotorMonkey::parseError(
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