#include "frc846/control/hardware/TalonFX_interm.h"

namespace frc846::control::hardware {

bool TalonFX_interm::VerifyConnected() { return talon_.IsAlive(); }

TalonFX_interm::TalonFX_interm(
    int can_id, std::string bus, units::millisecond_t max_wait_time)
    : talon_(can_id, bus), max_wait_time_(max_wait_time) {}

void TalonFX_interm::Tick() {
  ctre::phoenix::StatusCode last_status_code = ctre::phoenix::StatusCode::OK;
  if (double* dc = std::get_if<double>(&last_command_)) {
    ctre::phoenix6::controls::DutyCycleOut dc_msg{*dc};
    dc_msg.WithUpdateFreqHz(0_Hz);
    last_status_code = talon_.SetControl(dc_msg);
  } else if (units::radians_per_second_t* vel =
                 std::get_if<units::radians_per_second_t>(&last_command_)) {
    ctre::phoenix6::controls::VelocityVoltage vel_msg{*vel};
    vel_msg.WithUpdateFreqHz(0_Hz);
    last_status_code = talon_.SetControl(vel_msg);
  } else if (units::radian_t* pos =
                 std::get_if<units::radian_t>(&last_command_)) {
    ctre::phoenix6::controls::PositionVoltage pos_msg{*pos};
    pos_msg.WithUpdateFreqHz(0_Hz);
    last_status_code = talon_.SetControl(pos_msg);
  }
  last_error_ = getErrorCode(last_status_code);
}

void TalonFX_interm::SetInverted(bool inverted) {
  ctre::phoenix6::configs::MotorOutputConfigs motor_output_config{};
  motor_output_config.WithInverted(inverted);
  last_error_ =
      getErrorCode(talon_.GetConfigurator().Apply(motor_output_config));
}

void TalonFX_interm::SetNeutralMode(bool brake_mode) {
  talon_.SetNeutralMode(brake_mode
                            ? ctre::phoenix6::signals::NeutralModeValue::Brake
                            : ctre::phoenix6::signals::NeutralModeValue::Coast);
}

void TalonFX_interm::SetCurrentLimit(units::ampere_t current_limit) {
  ctre::phoenix6::configs::CurrentLimitsConfigs configs{};
  configs.WithSupplyCurrentLimitEnable(false);
  configs.WithStatorCurrentLimitEnable(true);
  configs.WithStatorCurrentLimit(current_limit);
  last_error_ = getErrorCode(talon_.GetConfigurator().Apply(configs));
}

void TalonFX_interm::SetSoftLimits(
    units::radian_t forward_limit, units::radian_t reverse_limit) {
  ctre::phoenix6::configs::SoftwareLimitSwitchConfigs configs{};
  configs.WithForwardSoftLimitEnable(true);
  configs.WithForwardSoftLimitThreshold(forward_limit);
  configs.WithReverseSoftLimitEnable(true);
  configs.WithReverseSoftLimitThreshold(reverse_limit);
  last_error_ = getErrorCode(talon_.GetConfigurator().Apply(configs));
}

void TalonFX_interm::SetVoltageCompensation(
    units::volt_t voltage_compensation) {
  ctre::phoenix6::configs::VoltageConfigs configs{};
  configs.WithPeakForwardVoltage(voltage_compensation);
  configs.WithPeakReverseVoltage(-voltage_compensation);
  last_error_ = getErrorCode(talon_.GetConfigurator().Apply(configs));
}

void TalonFX_interm::SetGains(frc846::control::base::MotorGains gains) {
  gains_ = gains;
  ctre::phoenix6::configs::Slot0Configs configs{};
  configs.WithKP(gains_.kP).WithKI(gains_.kI).WithKD(gains_.kD).WithKS(
      gains_.kFF);
  last_error_ = getErrorCode(talon_.GetConfigurator().Apply(configs));
}

void TalonFX_interm::WriteDC(double duty_cycle) { last_command_ = duty_cycle; }
void TalonFX_interm::WriteVelocity(units::radians_per_second_t velocity) {
  last_command_ = velocity;
}
void TalonFX_interm::WritePosition(units::radian_t position) {
  last_command_ = position;
}

void TalonFX_interm::EnableStatusFrames(
    std::vector<frc846::control::config::StatusFrame> frames) {
  last_error_ =
      getErrorCode(talon_.OptimizeBusUtilization(0_Hz, max_wait_time_));
  if (last_error_ != ControllerErrorCodes::kAllOK) { return; }
  for (auto frame : frames) {
    ctre::phoenix::StatusCode last_status_code = ctre::phoenix::StatusCode::OK;
    if (frame == frc846::control::config::StatusFrame::kCurrentFrame) {
      last_status_code = talon_.GetSupplyCurrent().SetUpdateFrequency(10_Hz);
    } else if (frame == frc846::control::config::StatusFrame::kPositionFrame) {
      last_status_code =
          talon_.GetPosition().SetUpdateFrequency(50_Hz, max_wait_time_);
    } else if (frame == frc846::control::config::StatusFrame::kVelocityFrame) {
      last_status_code = talon_.GetVelocity().SetUpdateFrequency(50_Hz);
    }
    last_error_ = getErrorCode(last_status_code);
    if (last_error_ != ControllerErrorCodes::kAllOK) return;
  }
}

bool TalonFX_interm::IsDuplicateControlMessage(double duty_cycle) {
  if (double* dc = std::get_if<double>(&last_command_)) {
    return *dc == duty_cycle;
  }
  return false;
}
bool TalonFX_interm::IsDuplicateControlMessage(
    units::radians_per_second_t velocity) {
  if (units::radians_per_second_t* vel =
          std::get_if<units::radians_per_second_t>(&last_command_)) {
    return *vel == velocity;
  }
  return false;
}
bool TalonFX_interm::IsDuplicateControlMessage(units::radian_t position) {
  if (units::radian_t* pos = std::get_if<units::radian_t>(&last_command_)) {
    return *pos == position;
  }
  return false;
}

void TalonFX_interm::ZeroEncoder(units::radian_t position) {
  last_error_ = getErrorCode(talon_.SetPosition(position, max_wait_time_));
}

units::radians_per_second_t TalonFX_interm::GetVelocity() {
  return talon_.GetVelocity().GetValue();
}
units::radian_t TalonFX_interm::GetPosition() {
  return talon_.GetPosition().GetValue();
}
units::ampere_t TalonFX_interm::GetCurrent() {
  return talon_.GetSupplyCurrent().GetValue();
}

ControllerErrorCodes TalonFX_interm::GetLastErrorCode() {
  ControllerErrorCodes toReturn = last_error_;
  last_error_ = ControllerErrorCodes::kAllOK;
  return toReturn;
}

// TODO: Expand list of errors
frc846::control::hardware::ControllerErrorCodes TalonFX_interm::getErrorCode(
    ctre::phoenix::StatusCode code) {
  switch (code) {
  case ctre::phoenix::StatusCode::OK: return ControllerErrorCodes::kAllOK;
  case ctre::phoenix::StatusCode::InvalidDeviceSpec:
    return ControllerErrorCodes::kDeviceDisconnected;
  case ctre::phoenix::StatusCode::ConfigFailed:
    return ControllerErrorCodes::kConfigFailed;
  case ctre::phoenix::StatusCode::ApiTooOld:
    return ControllerErrorCodes::kVersionMismatch;
  }

  if (code.IsWarning()) {
    return ControllerErrorCodes::kWarning;
  } else if (code.IsError()) {
    return ControllerErrorCodes::kError;
  }
  return ControllerErrorCodes::kAllOK;
}

}  // namespace frc846::control::hardware