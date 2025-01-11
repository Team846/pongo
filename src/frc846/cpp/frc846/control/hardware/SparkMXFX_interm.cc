#include "frc846/control/hardware/SparkMXFX_interm.h"

#include "frc846/wpilib/util.h"

namespace frc846::control::hardware {

#define set_last_error(code) last_error_ = getErrorCode(code)
#define set_last_error_and_break(code) \
  set_last_error(code);                \
  if (last_error_ != ControllerErrorCodes::kAllOK) return

bool SparkMXFX_interm::VerifyConnected() {
  if (esc_ == nullptr) return false;
  esc_->GetFirmwareVersion();
  return esc_->GetFirmwareVersion() != 0;
}

// This will make all soft configs reset
#define RESET rev::spark::SparkBase::ResetMode::kResetSafeParameters
// This will make all soft configs not reset
#define DONT_RESET rev::spark::SparkBase::ResetMode::kNoResetSafeParameters

// This will make configs persist through power cycles
#define PERSIST_PARAMS rev::spark::SparkBase::PersistMode::kPersistParameters
// Configs will not persist through power cycles
#define DONT_PERSIST_PARAMS \
  rev::spark::SparkBase::PersistMode::kNoPersistParameters

SparkMXFX_interm::SparkMXFX_interm(int can_id,
    units::millisecond_t max_wait_time, bool is_controller_spark_flex) {
  esc_ = is_controller_spark_flex
             ? static_cast<rev::spark::SparkBase*>(new rev::spark::SparkFlex{
                   can_id, rev::spark::SparkFlex::MotorType::kBrushless})
             : new rev::spark::SparkMax{
                   can_id, rev::spark::SparkMax::MotorType::kBrushless};
  encoder_ = new rev::spark::SparkRelativeEncoder{esc_->GetEncoder()};
  pid_controller_ = new rev::spark::SparkClosedLoopController{
      esc_->GetClosedLoopController()};
  esc_->Configure(configs, RESET, PERSIST_PARAMS);
  esc_->SetCANTimeout(max_wait_time.to<int>());
}

void SparkMXFX_interm::Tick() {
  rev::REVLibError last_status_code = rev::REVLibError::kOk;
  if (double* dc = std::get_if<double>(&last_command_)) {
    last_status_code = pid_controller_->SetReference(
        *dc, rev::spark::SparkBase::ControlType::kDutyCycle);
  } else if (units::radians_per_second_t* vel =
                 std::get_if<units::radians_per_second_t>(&last_command_)) {
    last_status_code = pid_controller_->SetReference(
        vel->to<double>(), rev::spark::SparkBase::ControlType::kVelocity);
  } else if (units::radian_t* pos =
                 std::get_if<units::radian_t>(&last_command_)) {
    last_status_code = pid_controller_->SetReference(
        pos->to<double>(), rev::spark::SparkBase::ControlType::kPosition);
  }
  set_last_error(last_status_code);
}

void SparkMXFX_interm::SetInverted(bool inverted) {
  configs.Inverted(inverted);
  set_last_error(esc_->Configure(configs, DONT_RESET, PERSIST_PARAMS));
}

void SparkMXFX_interm::SetNeutralMode(bool brake_mode) {
  configs.SetIdleMode(brake_mode
                          ? rev::spark::SparkBaseConfig::IdleMode::kBrake
                          : rev::spark::SparkBaseConfig::IdleMode::kCoast);
  set_last_error(esc_->Configure(configs, DONT_RESET, PERSIST_PARAMS));
}

void SparkMXFX_interm::SetCurrentLimit(units::ampere_t current_limit) {
  configs.SmartCurrentLimit(current_limit.to<int>());
  set_last_error(esc_->Configure(configs, DONT_RESET, PERSIST_PARAMS));
}

void SparkMXFX_interm::SetSoftLimits(
    units::radian_t forward_limit, units::radian_t reverse_limit) {
  configs.softLimit.ForwardSoftLimitEnabled(true)
      .ForwardSoftLimit(forward_limit.to<double>())
      .ReverseSoftLimitEnabled(true)
      .ReverseSoftLimit(reverse_limit.to<double>());
  set_last_error(esc_->Configure(configs, DONT_RESET, PERSIST_PARAMS));
}

void SparkMXFX_interm::SetVoltageCompensation(
    units::volt_t voltage_compensation) {
  configs.VoltageCompensation(voltage_compensation.to<double>());
  set_last_error(esc_->Configure(configs, DONT_RESET, PERSIST_PARAMS));
}

void SparkMXFX_interm::SetGains(frc846::control::base::MotorGains gains) {
  gains_ = gains;
  configs.closedLoop.Pidf(gains_.kP, gains_.kI, gains_.kD, gains_.kFF);
  set_last_error(esc_->Configure(configs, DONT_RESET, PERSIST_PARAMS));
}

void SparkMXFX_interm::WriteDC(double duty_cycle) {
  last_command_ = duty_cycle;
}
void SparkMXFX_interm::WriteVelocity(units::radians_per_second_t velocity) {
  last_command_ = velocity;
}
void SparkMXFX_interm::WritePosition(units::radian_t position) {
  last_command_ = position;
}

void SparkMXFX_interm::EnableStatusFrames(
    std::vector<frc846::control::config::StatusFrame> frames) {
  rev::REVLibError last_status_code;

  if (vector_has(frames, config::StatusFrame::kLeader) ||
      vector_has(frames, config::StatusFrame::kFaultFrame)) {
    configs.signals.FaultsPeriodMs(20);
    configs.signals.FaultsAlwaysOn(true);
  } else {
    configs.signals.FaultsPeriodMs(32767);
    configs.signals.FaultsAlwaysOn(false);
  }

  if (vector_has(frames, config::StatusFrame::kVelocityFrame) ||
      vector_has(frames, config::StatusFrame::kCurrentFrame)) {
    configs.signals.PrimaryEncoderVelocityPeriodMs(20);
    configs.signals.PrimaryEncoderVelocityAlwaysOn(true);
  } else {
    configs.signals.PrimaryEncoderVelocityPeriodMs(32767);
    configs.signals.PrimaryEncoderVelocityAlwaysOn(false);
  }

  if (vector_has(frames, config::StatusFrame::kPositionFrame)) {
    configs.signals.PrimaryEncoderPositionPeriodMs(20);
    configs.signals.PrimaryEncoderPositionAlwaysOn(true);
  } else {
    configs.signals.PrimaryEncoderPositionPeriodMs(32767);
    configs.signals.PrimaryEncoderPositionAlwaysOn(false);
  }

  if (vector_has(frames, config::StatusFrame::kSensorFrame)) {
    configs.signals.AnalogPositionPeriodMs(20);
    configs.signals.AnalogPositionAlwaysOn(true);
  } else {
    configs.signals.AnalogPositionPeriodMs(32767);
    configs.signals.AnalogPositionAlwaysOn(false);
  }

  set_last_error_and_break(
      esc_->Configure(configs, DONT_RESET, PERSIST_PARAMS));
}

bool SparkMXFX_interm::IsDuplicateControlMessage(double duty_cycle) {
  if (double* dc = std::get_if<double>(&last_command_)) {
    return *dc == duty_cycle;
  }
  return false;
}
bool SparkMXFX_interm::IsDuplicateControlMessage(
    units::radians_per_second_t velocity) {
  if (units::radians_per_second_t* vel =
          std::get_if<units::radians_per_second_t>(&last_command_)) {
    return *vel == velocity;
  }
  return false;
}
bool SparkMXFX_interm::IsDuplicateControlMessage(units::radian_t position) {
  if (units::radian_t* pos = std::get_if<units::radian_t>(&last_command_)) {
    return *pos == position;
  }
  return false;
}

void SparkMXFX_interm::ZeroEncoder(units::radian_t position) {
  set_last_error(encoder_->SetPosition(position.to<double>()));
}

units::radians_per_second_t SparkMXFX_interm::GetVelocity() {
  return units::make_unit<units::radians_per_second_t>(encoder_->GetVelocity());
}
units::radian_t SparkMXFX_interm::GetPosition() {
  return units::make_unit<units::radian_t>(encoder_->GetPosition());
}
units::ampere_t SparkMXFX_interm::GetCurrent() {
  return units::make_unit<units::ampere_t>(esc_->GetOutputCurrent());
}

ControllerErrorCodes SparkMXFX_interm::GetLastErrorCode() {
  ControllerErrorCodes toReturn = last_error_;
  last_error_ = ControllerErrorCodes::kAllOK;
  return toReturn;
}

frc846::control::hardware::ControllerErrorCodes SparkMXFX_interm::getErrorCode(
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

SparkMAX_interm::SparkMAX_interm(int can_id, units::millisecond_t max_wait_time)
    : SparkMXFX_interm(can_id, max_wait_time, false) {}

SparkFLEX_interm::SparkFLEX_interm(
    int can_id, units::millisecond_t max_wait_time)
    : SparkMXFX_interm(can_id, max_wait_time, true) {}

}  // namespace frc846::control::hardware