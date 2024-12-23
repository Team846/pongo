#include "frc846/control/hardware/SparkMAX_interm.h"

namespace frc846::control::hardware {

SparkMAX_interm::SparkMAX_interm(int can_id, units::millisecond_t max_wait_time,
    rev::CANSparkBase::MotorType motor_type)
    : SparkMax_(can_id, motor_type) {
  SparkMax_.SetCANTimeout(max_wait_time.to<int>());
}

void SparkMAX_interm::Tick() {
  rev::REVLibError last_status_code = rev::REVLibError::kOk;
  if (double* dc = std::get_if<double>(&last_command_)) {
    last_status_code = SparkMax_.GetPIDController().SetReference(
        *dc, rev::CANSparkBase::ControlType::kDutyCycle);
  } else if (units::radians_per_second_t* vel =
                 std::get_if<units::radians_per_second_t>(&last_command_)) {
    last_status_code = SparkMax_.GetPIDController().SetReference(
        vel->to<double>(), rev::CANSparkBase::ControlType::kVelocity);
  } else if (units::radian_t* pos =
                 std::get_if<units::radian_t>(&last_command_)) {
    last_status_code = SparkMax_.GetPIDController().SetReference(
        pos->to<double>(), rev::CANSparkBase::ControlType::kPosition);
  }
  last_error_ = getErrorCode(last_status_code);
}

void SparkMAX_interm::SetInverted(bool inverted) {
  SparkMax_.SetInverted(inverted);
}
void SparkMAX_interm::SetNeutralMode(bool brake_mode) {
  SparkMax_.SetIdleMode(brake_mode ? rev::CANSparkBase::IdleMode::kBrake
                                   : rev::CANSparkBase::IdleMode::kCoast);
}
void SparkMAX_interm::SetCurrentLimit(units::ampere_t current_limit) {
  last_error_ =
      getErrorCode(SparkMax_.SetSmartCurrentLimit(current_limit.to<double>()));
}

void SparkMAX_interm::SetSoftLimits(
    units::radian_t forward_limit, units::radian_t reverse_limit) {
  last_error_ = getErrorCode(SparkMax_.EnableSoftLimit(
      rev::CANSparkBase::SoftLimitDirection::kForward, true));
  last_error_ = getErrorCode(
      SparkMax_.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward,
          forward_limit.to<double>()));
  last_error_ = getErrorCode(SparkMax_.EnableSoftLimit(
      rev::CANSparkBase::SoftLimitDirection::kReverse, true));
  last_error_ = getErrorCode(
      SparkMax_.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse,
          reverse_limit.to<double>()));
}

void SparkMAX_interm::SetVoltageCompensation(
    units::volt_t voltage_compensation) {
  last_error_ = getErrorCode(
      SparkMax_.EnableVoltageCompensation(voltage_compensation.to<double>()));
}

void SparkMAX_interm::SetGains(frc846::control::base::MotorGains gains) {
  gains_ = gains;
  rev::SparkPIDController pid_controller = SparkMax_.GetPIDController();
  last_error_ = getErrorCode(pid_controller.SetP(gains_.kP));
  last_error_ = getErrorCode(pid_controller.SetI(gains_.kI));
  last_error_ = getErrorCode(pid_controller.SetD(gains_.kD));
  last_error_ = getErrorCode(pid_controller.SetFF(gains_.kFF));
}

void SparkMAX_interm::WriteDC(double duty_cycle) { last_command_ = duty_cycle; }
void SparkMAX_interm::WriteVelocity(units::radians_per_second_t velocity) {
  last_command_ = velocity;
}
void SparkMAX_interm::WritePosition(units::radian_t position) {
  last_command_ = position;
}

void SparkMAX_interm::EnableStatusFrames(
    std::vector<frc846::control::config::StatusFrame> frames) {
  if (last_error_ != ControllerErrorCodes::kAllOK) { return; }
  for (auto frame : frames) {
    rev::REVLibError last_status_code = rev::REVLibError::kOk;
    if (frame == frc846::control::config::StatusFrame::kCurrentFrame) {
      last_status_code = SparkMax_.SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 100);
    } else if (frame == frc846::control::config::StatusFrame::kPositionFrame) {
      last_status_code = SparkMax_.SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 20);
    } else if (frame == frc846::control::config::StatusFrame::kVelocityFrame) {
      last_status_code = SparkMax_.SetPeriodicFramePeriod(
          rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 20);
    }
    last_error_ = getErrorCode(last_status_code);
    if (last_error_ != ControllerErrorCodes::kAllOK) return;
  }
}

bool SparkMAX_interm::IsDuplicateControlMessage(double duty_cycle) {
  if (double* dc = std::get_if<double>(&last_command_)) {
    return *dc == duty_cycle;
  }
  return false;
}
bool SparkMAX_interm::IsDuplicateControlMessage(
    units::radians_per_second_t velocity) {
  if (units::radians_per_second_t* vel =
          std::get_if<units::radians_per_second_t>(&last_command_)) {
    return *vel == velocity;
  }
  return false;
}
bool SparkMAX_interm::IsDuplicateControlMessage(units::radian_t position) {
  if (units::radian_t* pos = std::get_if<units::radian_t>(&last_command_)) {
    return *pos == position;
  }
  return false;
}

void SparkMAX_interm::ZeroEncoder(units::radian_t position) {
  last_error_ = getErrorCode(
      SparkMax_.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)
          .SetPosition(position.to<double>()));
}

units::radians_per_second_t SparkMAX_interm::GetVelocity() {
  return units::make_unit<units::radians_per_second_t>(
      SparkMax_.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)
          .GetVelocity());
}
units::radian_t SparkMAX_interm::GetPosition() {
  return units::make_unit<units::radian_t>(
      SparkMax_.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)
          .GetPosition());
}
units::ampere_t SparkMAX_interm::GetCurrent() {
  return units::make_unit<units::ampere_t>(SparkMax_.GetOutputCurrent());
}

ControllerErrorCodes SparkMAX_interm::GetLastErrorCode() { return last_error_; }

frc846::control::hardware::ControllerErrorCodes SparkMAX_interm::getErrorCode(
    rev::REVLibError code) {
  switch (code) {
  case rev::REVLibError::kOk: return ControllerErrorCodes::kAllOK;
  case rev::REVLibError::kInvalidCANId:
    return ControllerErrorCodes::kInvalidCANID;
  case rev::REVLibError::kTimeout: return ControllerErrorCodes::kTimeout;
  case rev::REVLibError::kHALError: return ControllerErrorCodes::kHALError;
  case rev::REVLibError::kFollowConfigMismatch:
    return ControllerErrorCodes::kFollowingError;
  case rev::REVLibError::kCANDisconnected:
    return ControllerErrorCodes::kCANDisconnected;
  case rev::REVLibError::kCantFindFirmware:
  case rev::REVLibError::kFirmwareTooOld:
  case rev::REVLibError::kFirmwareTooNew:
    return ControllerErrorCodes::kVersionMismatch;
  case rev::REVLibError::kParamInvalidID:
  case rev::REVLibError::kParamMismatchType:
  case rev::REVLibError::kParamAccessMode:
  case rev::REVLibError::kParamInvalid:
  case rev::REVLibError::kSparkMaxDataPortAlreadyConfiguredDifferently:
    return ControllerErrorCodes::kConfigFailed;
  case rev::REVLibError::kError: return ControllerErrorCodes::kError;
  case rev::REVLibError::kParamNotImplementedDeprecated:
  case rev::REVLibError::kNotImplemented: return ControllerErrorCodes::kWarning;
  }
  return ControllerErrorCodes::kAllOK;
}

}  // namespace frc846::control::hardware