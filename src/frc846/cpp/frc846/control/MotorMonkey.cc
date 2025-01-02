#include "frc846/control/MotorMonkey.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>

#include <string>

#include "frc846/control/hardware/SparkMXFX_interm.h"
#include "frc846/control/hardware/TalonFX_interm.h"
#include "frc846/control/hardware/simulation/MCSimulator.h"
#include "frc846/control/hardware/simulation/SIMLEVEL.h"
#include "frc846/math/collection.h"

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
std::map<size_t, bool> MotorMonkey::slot_id_to_sim_{};

frc846::control::hardware::IntermediateController*
    MotorMonkey::controller_registry[CONTROLLER_REGISTRY_SIZE]{};

frc846::control::base::MotorGains
    MotorMonkey::gains_registry[CONTROLLER_REGISTRY_SIZE]{};

units::newton_meter_t MotorMonkey::load_registry[CONTROLLER_REGISTRY_SIZE]{};

units::volt_t MotorMonkey::battery_voltage{0_V};

void MotorMonkey::Tick() {
  battery_voltage = frc::RobotController::GetBatteryVoltage();
  // TODO: Improve battery voltage estimation for simulation

  for (size_t i = 0; i < CONTROLLER_REGISTRY_SIZE; i++) {
    if (controller_registry[i] != nullptr) {
      controller_registry[i]->Tick();
      if (slot_id_to_sim_[i]) {
        simulation::MCSimulator* sim =
            dynamic_cast<simulation::MCSimulator*>(controller_registry[i]);
        sim->SetBatteryVoltage(battery_voltage);
        sim->SetLoad(load_registry[i]);
      }
    }
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
  slot_id_to_sim_[slot_id] = false;

  frc846::control::hardware::IntermediateController* this_controller = nullptr;

  if (frc::RobotBase::IsSimulation() &&
      MOTOR_SIM_LEVEL == MOTOR_SIM_LEVEL_SIM_HARDWARE) {
    slot_id_to_sim_[slot_id] = true;
    this_controller = controller_registry[slot_id] =
        new frc846::control::simulation::MCSimulator{
            frc846::control::base::MotorSpecificationPresets::get(type),
            params.circuit_resistance, params.rotational_inertia};
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

void MotorMonkey::EnableStatusFrames(
    size_t slot_id, std::vector<frc846::control::config::StatusFrame> frames) {
  CHECK_SLOT_ID();

  SMART_RETRY(controller_registry[slot_id]->EnableStatusFrames(frames),
      "EnableStatusFrames");
  LOG_IF_ERROR("EnableStatusFrames");
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

  controller_registry[slot_id]->WriteDC(duty_cycle);
  LOG_IF_ERROR("WriteDC");
}

void MotorMonkey::WriteVelocity(
    size_t slot_id, units::radians_per_second_t velocity) {
  CHECK_SLOT_ID();

  controller_registry[slot_id]->WriteVelocity(velocity);
  LOG_IF_ERROR("WriteVelocity");
}

void MotorMonkey::WritePosition(size_t slot_id, units::radian_t position) {
  CHECK_SLOT_ID();

  controller_registry[slot_id]->WritePosition(position);
  LOG_IF_ERROR("WritePosition");
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