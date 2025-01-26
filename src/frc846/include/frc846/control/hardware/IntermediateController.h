#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/voltage.h>

#include <vector>

#include "frc846/control/base/motor_control_base.h"
#include "frc846/control/base/motor_gains.h"
#include "frc846/control/config/status_frames.h"

namespace frc846::control::hardware {

enum ControllerErrorCodes {
  kAllOK,
  kInvalidCANID,
  kVersionMismatch,
  kHALError,
  kFollowingError,
  kCANDisconnected,
  kCANMessageStale,
  kDeviceDisconnected,
  kConfigFailed,
  kTimeout,
  kWarning,
  kError,
};

class IntermediateController {
public:
  IntermediateController() = default;
  ~IntermediateController() = default;

  virtual void Tick() = 0;

  virtual void SetInverted(bool inverted) = 0;
  virtual void SetNeutralMode(bool brake_mode) = 0;
  virtual void SetCurrentLimit(units::ampere_t current_limit) = 0;

  virtual void SetSoftLimits(
      units::radian_t forward_limit, units::radian_t reverse_limit) = 0;

  virtual void SetVoltageCompensation(units::volt_t voltage_compensation) = 0;

  virtual void SetGains(frc846::control::base::MotorGains gains) = 0;

  virtual void WriteDC(double duty_cycle) = 0;
  virtual void WriteVelocity(units::radians_per_second_t velocity) = 0;
  virtual void WritePosition(units::radian_t position) = 0;

  virtual void EnableStatusFrames(
      std::vector<frc846::control::config::StatusFrame> frames) = 0;

  virtual bool IsDuplicateControlMessage(double duty_cycle) = 0;
  virtual bool IsDuplicateControlMessage(
      units::radians_per_second_t velocity) = 0;
  virtual bool IsDuplicateControlMessage(units::radian_t position) = 0;

  virtual void ZeroEncoder(units::radian_t position) = 0;

  virtual units::radians_per_second_t GetVelocity() = 0;
  virtual units::radian_t GetPosition() = 0;
  virtual units::ampere_t GetCurrent() = 0;

  virtual void ConfigForwardLimitSwitch(
      bool stop_motor, frc846::control::base::LimitSwitchDefaultState type) = 0;

  virtual void ConfigReverseLimitSwitch(
      bool stop_motor, frc846::control::base::LimitSwitchDefaultState type) = 0;

  virtual bool GetForwardLimitSwitchState() = 0;
  virtual bool GetReverseLimitSwitchState() = 0;

  virtual units::volt_t GetAnalogDeviceOutput() = 0;

  virtual ControllerErrorCodes GetLastErrorCode() = 0;

  virtual bool VerifyConnected() = 0;  // changed
};

}  // namespace frc846::control::hardware