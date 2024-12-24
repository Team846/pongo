#include "frc846/control/hardware/SparkMAX_interm.h"
#include "frc846/wpilib/util.h"

namespace frc846::control::hardware {

#define set_last_error(code) last_error_ = getErrorCode(code)
#define set_last_error_and_break(code) \
  set_last_error(code);                \
  if (last_error_ != ControllerErrorCodes::kAllOK) return

SparkMAX_interm::SparkMAX_interm(int can_id, units::millisecond_t max_wait_time,
    rev::CANSparkBase::MotorType motor_type)
    : SparkMax_(can_id, motor_type),
      encoder_(SparkMax_.GetEncoder(
          rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      pid_controller_(SparkMax_.GetPIDController()) {
  SparkMax_.SetCANTimeout(max_wait_time.to<int>());
}

void SparkMAX_interm::Tick() {
  rev::REVLibError last_status_code = rev::REVLibError::kOk;
  if (double* dc = std::get_if<double>(&last_command_)) {
    last_status_code = pid_controller_.SetReference(
        *dc, rev::CANSparkBase::ControlType::kDutyCycle);
  } else if (units::radians_per_second_t* vel =
                 std::get_if<units::radians_per_second_t>(&last_command_)) {
    last_status_code = pid_controller_.SetReference(
        vel->to<double>(), rev::CANSparkBase::ControlType::kVelocity);
  } else if (units::radian_t* pos =
                 std::get_if<units::radian_t>(&last_command_)) {
    last_status_code = pid_controller_.SetReference(
        pos->to<double>(), rev::CANSparkBase::ControlType::kPosition);
  }
  set_last_error(last_status_code);
}

void SparkMAX_interm::SetInverted(bool inverted) {
  SparkMax_.SetInverted(inverted);
}

void SparkMAX_interm::SetNeutralMode(bool brake_mode) {
  set_last_error(
      SparkMax_.SetIdleMode(brake_mode ? rev::CANSparkBase::IdleMode::kBrake
                                       : rev::CANSparkBase::IdleMode::kCoast));
}

void SparkMAX_interm::SetCurrentLimit(units::ampere_t current_limit) {
  set_last_error(SparkMax_.SetSmartCurrentLimit(current_limit.to<int>()));
}

void SparkMAX_interm::SetSoftLimits(
    units::radian_t forward_limit, units::radian_t reverse_limit) {
  set_last_error_and_break(SparkMax_.EnableSoftLimit(
      rev::CANSparkBase::SoftLimitDirection::kForward, true));
  set_last_error_and_break(
      SparkMax_.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward,
          forward_limit.to<double>()));
  set_last_error_and_break(SparkMax_.EnableSoftLimit(
      rev::CANSparkBase::SoftLimitDirection::kReverse, true));
  set_last_error_and_break(
      SparkMax_.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse,
          reverse_limit.to<double>()));
}

void SparkMAX_interm::SetVoltageCompensation(
    units::volt_t voltage_compensation) {
  set_last_error(
      SparkMax_.EnableVoltageCompensation(voltage_compensation.to<double>()));
}

void SparkMAX_interm::SetGains(frc846::control::base::MotorGains gains) {
  gains_ = gains;
  set_last_error_and_break(pid_controller_.SetP(gains_.kP));
  set_last_error_and_break(pid_controller_.SetI(gains_.kI));
  set_last_error_and_break(pid_controller_.SetD(gains_.kD));
  set_last_error_and_break(pid_controller_.SetFF(gains_.kFF));
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
  rev::REVLibError last_status_code = rev::REVLibError::kOk;

  if (vector_has(frames, config::StatusFrame::kLeader) ||
      vector_has(frames, config::StatusFrame::kFaultFrame)) {
    last_status_code = SparkMax_.SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 20);
  } else {
    last_status_code = SparkMax_.SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 65535);
  }

  set_last_error_and_break(last_status_code);

  if (vector_has(frames, config::StatusFrame::kVelocityFrame) ||
      vector_has(frames, config::StatusFrame::kCurrentFrame)) {
    last_status_code = SparkMax_.SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 20);
  } else {
    last_status_code = SparkMax_.SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 65535);
  }

  set_last_error_and_break(last_status_code);

  if (vector_has(frames, config::StatusFrame::kPositionFrame)) {
    last_status_code = SparkMax_.SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 20);
  }

  set_last_error_and_break(last_status_code);

  if (vector_has(frames, config::StatusFrame::kSensorFrame)) {
    last_status_code = SparkMax_.SetPeriodicFramePeriod(
        rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 20);
  }

  set_last_error_and_break(last_status_code);
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
  set_last_error(encoder_.SetPosition(position.to<double>()));
}

units::radians_per_second_t SparkMAX_interm::GetVelocity() {
  return units::make_unit<units::radians_per_second_t>(encoder_.GetVelocity());
}
units::radian_t SparkMAX_interm::GetPosition() {
  return units::make_unit<units::radian_t>(encoder_.GetPosition());
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
  case rev::REVLibError::kDuplicateCANId:
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
  case rev::REVLibError::kNotImplemented:
  case rev::REVLibError::kUnknown:
  default: return ControllerErrorCodes::kWarning;
  }
}

}  // namespace frc846::control::hardware