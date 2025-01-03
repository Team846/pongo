#include "frc846/control/HigherMotorController.h"

#include "frc846/control/MotorMonkey.h"
#include "frc846/control/calculators/CurrentTorqueCalculator.h"

namespace frc846::control {

HigherMotorController::HigherMotorController(
    frc846::control::base::MotorMonkeyType mmtype,
    frc846::control::config::MotorConstructionParameters params)
    : mmtype_(mmtype), constr_params_(params) {}

void HigherMotorController::Setup() {
  slot_id_ = frc846::control::MotorMonkey::ConstructController(
      mmtype_, constr_params_);
}

bool HigherMotorController::VerifyConnected() {
  return frc846::control::MotorMonkey::VerifyConnected();
}

void HigherMotorController::SetGains(frc846::control::base::MotorGains gains) {
  gains_ = gains;
  frc846::control::MotorMonkey::SetGains(slot_id_, gains_);
}

void HigherMotorController::SetLoad(units::newton_meter_t load) {
  load_ = load;
  frc846::control::MotorMonkey::SetLoad(slot_id_, load_);
}

void HigherMotorController::WriteDC(double duty_cycle) {
  if (soft_limits_.has_value()) {
    duty_cycle = soft_limits_.value().LimitDC(duty_cycle, GetPosition());
  }
  MotorMonkey::WriteDC(slot_id_, duty_cycle);
}

void HigherMotorController::WriteCurrent(units::ampere_t current) {
  double dc_target =
      frc846::control::calculators::CurrentTorqueCalculator::current_control(
          current, GetVelocity(), MotorMonkey::GetBatteryVoltage(),
          constr_params_.circuit_resistance, mmtype_);
  WriteDC(dc_target);
}

void HigherMotorController::WriteTorque(units::newton_meter_t torque) {
  WriteCurrent(
      frc846::control::calculators::CurrentTorqueCalculator::torque_to_current(
          torque, mmtype_));
}

void HigherMotorController::WriteVelocity(
    units::radians_per_second_t velocity) {
  double dc_target = gains_.calculate(
      (velocity - GetVelocity()).to<double>(), 0.0, 0.0, load_.to<double>());
  WriteDC(dc_target);
}

void HigherMotorController::WritePosition(units::radian_t position) {
  double dc_target = gains_.calculate((position - GetPosition()).to<double>(),
      0.0, GetVelocity().to<double>(), load_.to<double>());
  WriteDC(dc_target);
}

void HigherMotorController::WriteVelocityOnController(
    units::radians_per_second_t velocity) {
  if (soft_limits_.has_value()) {
    velocity = soft_limits_.value().LimitVelocity(velocity, GetPosition());
  }
  frc846::control::MotorMonkey::WriteVelocity(slot_id_, velocity);
}

void HigherMotorController::WritePositionOnController(
    units::radian_t position) {
  if (soft_limits_.has_value()) {
    position = soft_limits_.value().LimitPosition(position);
  }
  frc846::control::MotorMonkey::WritePosition(slot_id_, position);
}

units::radians_per_second_t HigherMotorController::GetVelocity() {
  return frc846::control::MotorMonkey::GetVelocity(slot_id_);
}

units::radian_t HigherMotorController::GetPosition() {
  return frc846::control::MotorMonkey::GetPosition(slot_id_);
}

units::ampere_t HigherMotorController::GetCurrent() {
  return frc846::control::MotorMonkey::GetCurrent(slot_id_);
}

void HigherMotorController::SetPosition(units::radian_t position) {
  frc846::control::MotorMonkey::ZeroEncoder(slot_id_, position);
}

void HigherMotorController::SetControllerSoftLimits(
    units::radian_t forward_limit, units::radian_t reverse_limit) {
  frc846::control::MotorMonkey::SetSoftLimits(
      slot_id_, forward_limit, reverse_limit);
}

void HigherMotorController::SetSoftLimits(
    frc846::control::config::SoftLimits soft_limits) {
  soft_limits_ = soft_limits;
}

void HigherMotorController::EnableStatusFrames(
    std::vector<frc846::control::config::StatusFrame> frames) {
  frc846::control::MotorMonkey::EnableStatusFrames(slot_id_, frames);
}

}  // namespace frc846::control
