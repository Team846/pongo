#include "frc846/control/MotorMonkey.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>

#include "frc846/control/hardware/TalonFX_interm.h"
#include "frc846/control/hardware/SparkMXFX_interm.h"

#include "frc846/math/collection.h"

// TODO: Add error handling
// TODO: Add retries
// TODO: Add dynamic can/power management
// TODO: Error for invalid slot ID

namespace frc846::control {

size_t MotorMonkey::slot_counter_{0};
std::map<size_t, frc846::control::base::MotorMonkeyType>
    MotorMonkey::slot_id_to_type_{};

frc846::control::hardware::IntermediateController*
    MotorMonkey::controller_registry[CONTROLLER_REGISTRY_SIZE]{};

frc846::control::base::MotorGains
    MotorMonkey::gains_registry[CONTROLLER_REGISTRY_SIZE]{};

units::newton_meter_t MotorMonkey::load_registry[CONTROLLER_REGISTRY_SIZE]{};

frc846::control::hardware::ControllerErrorCodes
    MotorMonkey::last_error_registry[CONTROLLER_REGISTRY_SIZE]{};

units::volt_t MotorMonkey::battery_voltage{0_V};

void MotorMonkey::Tick() {
  battery_voltage = frc::RobotController::GetBatteryVoltage();

  for (size_t i = 0; i < CONTROLLER_REGISTRY_SIZE; i++) {
    if (controller_registry[i] != nullptr) { controller_registry[i]->Tick(); }
  }
}

size_t MotorMonkey::ConstructController(
    frc846::control::base::MotorMonkeyType type,
    frc846::control::config::MotorConstructionParameters params) {
  slot_counter_++;
  slot_id_to_type_[slot_counter_] = type;

  frc846::control::hardware::IntermediateController* this_controller = nullptr;

  if (frc::RobotBase::IsSimulation()) {
    // TODO: implement sim controllers
  } else if (frc846::control::base::MotorMonkeyTypeHelper::is_talon_fx(type)) {
    this_controller = controller_registry[slot_counter_] =
        new frc846::control::hardware::TalonFX_interm{
            params.can_id, params.bus, params.max_wait_time};

  } else if (frc846::control::base::MotorMonkeyTypeHelper::is_spark_max(type)) {
    this_controller = controller_registry[slot_counter_] =
        new frc846::control::hardware::SparkMAX_interm{
            params.can_id, params.max_wait_time};
  } else if (frc846::control::base::MotorMonkeyTypeHelper::is_spark_flex(
                 type)) {
    this_controller = controller_registry[slot_counter_] =
        new frc846::control::hardware::SparkFLEX_interm{
            params.can_id, params.max_wait_time};
  }

  if (this_controller == nullptr) return slot_counter_;

  this_controller->SetInverted(params.inverted);
  this_controller->SetNeutralMode(params.brake_mode);
  this_controller->SetCurrentLimit(params.motor_current_limit);
  this_controller->SetVoltageCompensation(params.voltage_compensation);

  return slot_counter_;
}

units::volt_t MotorMonkey::GetBatteryVoltage() { return battery_voltage; }

void MotorMonkey::SetLoad(size_t slot_id, units::newton_meter_t load) {
  load_registry[slot_id] = load;

  if (controller_registry[slot_id] != nullptr) {
    // TODO: If is MCSim, cast then set load
    // controller_registry[slot_id]->SetLoad(load);
  }
}

void MotorMonkey::SetGains(
    size_t slot_id, frc846::control::base::MotorGains gains) {
  if (frc846::math::DEquals(gains_registry[slot_id].kP, gains.kP) &&
      frc846::math::DEquals(gains_registry[slot_id].kI, gains.kI) &&
      frc846::math::DEquals(gains_registry[slot_id].kD, gains.kD) &&
      frc846::math::DEquals(gains_registry[slot_id].kFF, gains.kFF)) {
    return;
  } else {
    gains_registry[slot_id] = gains;
  }

  if (controller_registry[slot_id] != nullptr) {
    controller_registry[slot_id]->SetGains(gains);
  }
}

void MotorMonkey::WriteDC(size_t slot_id, double duty_cycle) {
  if (controller_registry[slot_id] != nullptr) {
    controller_registry[slot_id]->WriteDC(duty_cycle);
  }
}

void MotorMonkey::WriteVelocity(
    size_t slot_id, units::radians_per_second_t velocity) {
  if (controller_registry[slot_id] != nullptr) {
    controller_registry[slot_id]->WriteVelocity(velocity);
  }
}

void MotorMonkey::WritePosition(size_t slot_id, units::radian_t position) {
  if (controller_registry[slot_id] != nullptr) {
    controller_registry[slot_id]->WritePosition(position);
  }
}

void MotorMonkey::ZeroEncoder(size_t slot_id, units::radian_t position) {
  if (controller_registry[slot_id] != nullptr) {
    controller_registry[slot_id]->ZeroEncoder(position);
  }
}

units::radians_per_second_t MotorMonkey::GetVelocity(size_t slot_id) {
  if (controller_registry[slot_id] != nullptr) {
    return controller_registry[slot_id]->GetVelocity();
  }
  return 0_rad_per_s;
}

units::radian_t MotorMonkey::GetPosition(size_t slot_id) {
  if (controller_registry[slot_id] != nullptr) {
    return controller_registry[slot_id]->GetPosition();
  }
  return 0_rad;
}

units::ampere_t MotorMonkey::GetCurrent(size_t slot_id) {
  if (controller_registry[slot_id] != nullptr) {
    return controller_registry[slot_id]->GetCurrent();
  }
  return 0_A;
}

void MotorMonkey::SetSoftLimits(size_t slot_id, units::radian_t forward_limit,
    units::radian_t reverse_limit) {
  if (controller_registry[slot_id] != nullptr) {
    controller_registry[slot_id]->SetSoftLimits(forward_limit, reverse_limit);
  }
}

}  // namespace frc846::control