#pragma once

#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>

#include <variant>

#include "IntermediateController.h"

namespace frc846::control::hardware {

/*
SparkMXFX_interm

A class which interacts with the REV API to control and get information from
SparkMAX or SparkFLEX hardware.
*/
class SparkMXFX_interm : public IntermediateController {
public:
  SparkMXFX_interm(int can_id, units::millisecond_t max_wait_time,
      bool is_controller_spark_flex);
  /*
  Tick()

  Updates the motor controller state based off the most recent motor state.
  */
  void Tick() override;

  /*
  SetInverted()

  Configure the motor's inversion state.

  @param inverted Choose to invert the motor's direction.
  */
  void SetInverted(bool inverted) override;

  /*
  SetNeutralMode()

  Set the motor's neutral mode to break or coast.

  @param brake_mode Enable break mode.
  */
  void SetNeutralMode(bool brake_mode) override;

  /*
  SetCurrentLimit()

  Set the motor's current limit.

  @param current_limit Maximum allowable current.
  */
  void SetCurrentLimit(units::ampere_t current_limit) override;

  /*
  SetSoftLimits()

  Set software limits for forward and reverse motion.

  @param forward_limit The forward position limit in radians.
  @param reverse_limit The reverse position limit in radians.
  */
  void SetSoftLimits(
      units::radian_t forward_limit, units::radian_t reverse_limit) override;

  /*
  SetVoltageCompensation()

  Enables voltage compensation for the motor controller.

  @param voltage_compensation Voltage to be applied as compensation.
  */
  void SetVoltageCompensation(units::volt_t voltage_compensation) override;

  /*
  SetGains()

  Set the PIDF gains for the motor.

  @param gains The PIDF control gains.
  */
  void SetGains(frc846::control::base::MotorGains gains) override;

  /*
  WriteDC()

  Sends a duty cycle control signal.

  @param duty_cycle The duty cycle to send.
  */
  void WriteDC(double duty_cycle) override;

  /*
  WriteVelocity()

  Sends a velocity control signal.

  @param velocity The velocity to send.
  */
  void WriteVelocity(units::radians_per_second_t velocity) override;

  /*
  WritePosition()

  Sends a position control signal.

  @param position The position to send.
  */
  void WritePosition(units::radian_t position) override;

  /*
  EnableStatusFrames()

  Enables specified status frames which are periodically updated

  @param frames the frames to be enabled
  */
  void EnableStatusFrames(
      std::vector<frc846::control::config::StatusFrame> frames,
      units::millisecond_t faults_ms = 20_ms,
      units::millisecond_t velocity_ms = 20_ms,
      units::millisecond_t encoder_position_ms = 20_ms,
      units::millisecond_t analog_position_ms = 20_ms) override;

  void OverrideStatusFramePeriod(frc846::control::config::StatusFrame frame,
      units::millisecond_t period) override;

  /*
  IsDuplicateControlMessage()

  Checks if the given duty cycle control message matches the last request.

  @param duty_cycle The duty cycle to check.
  @return boolean for if the message is a duplicate.
  */
  bool IsDuplicateControlMessage(double duty_cycle) override;

  /*
  IsDuplicateControlMessage()

  Checks if the given velocity control message matches the last request.

  @param velocity The velocity to check.
  @return boolean for if the message is a duplicate.
  */
  bool IsDuplicateControlMessage(units::radians_per_second_t velocity) override;

  /*
  IsDuplicateControlMessage()

  Checks if the given position control message matches the last request.

  @param position The position to check.
  @return boolean for if the message is a duplicate.
  */
  bool IsDuplicateControlMessage(units::radian_t position) override;

  /*
  ZeroEncoder()

  Zeros the encoder to a specified position.

  @param position The position to set.
  */
  void ZeroEncoder(units::radian_t position) override;

  /*
  VerifyHardware()

  Checks the TalonFX hardware and verifies it is functioning correctly.

  @return boolean indicating whether the hardware is verified successfully.
  */
  bool VerifyConnected() override;  // added

  /*
  GetVelocity()

  Gets the motor's current velocity.

  @return The velocity
  */
  units::radians_per_second_t GetVelocity() override;

  /*
  GetPosition()

  Gets the motor's current position.

  @return The position
  */
  units::radian_t GetPosition() override;

  /*
  GetCurrent()

  Gets the motor's current draw.

  @return The current
  */
  units::ampere_t GetCurrent() override;

  /*
  ConfigForwardLimitSwitch()

  Configures the forward limit switch.

  @param stop_motor Stop the motor when the limit switch is true.
  @param type The state which the limit switch is normally.
  */
  void ConfigForwardLimitSwitch(
      bool stop_motor, frc846::control::base::LimitSwitchDefaultState type);

  /*
  ConfigReverseLimitSwitch()

  Configures the reverse limit switch.

  @param stop_motor Stop the motor when the limit switch is true.
  @param type The state which the limit switch is normally.
  */
  void ConfigReverseLimitSwitch(
      bool stop_motor, frc846::control::base::LimitSwitchDefaultState type);

  /*
  GetForwardLimitSwitchState()

  Gets the limit switch's state.

  @return bool If the limit switch is enabled
  */
  bool GetForwardLimitSwitchState();

  /*
  GetForwardLimitSwitchState()

  Gets the limit switch's state.

  @return bool If the limit switch is enabled
  */
  bool GetReverseLimitSwitchState();

  /*
  GetLastErrorCode()

  Gets the last error code from the motor controller.

  @return The last error code.
  */
  ControllerErrorCodes GetLastErrorCode() override;

private:
  frc846::control::hardware::ControllerErrorCodes getErrorCode(
      rev::REVLibError code);

  rev::spark::SparkBaseConfig configs{};

  std::variant<units::turn_t, units::revolutions_per_minute_t, double>
      last_command_;
  frc846::control::base::MotorGains gains_;

  rev::spark::SparkBase* esc_;
  rev::spark::SparkRelativeEncoder* encoder_;
  rev::spark::SparkClosedLoopController* pid_controller_;

  frc846::control::hardware::ControllerErrorCodes last_error_;
};

/*
SparkMAX_interm

A class which interacts with the REV API to control and get information from
SparkMAX hardware. Derives from SparkMXFX_interm.
*/
class SparkMAX_interm : public SparkMXFX_interm {
public:
  SparkMAX_interm(int can_id, units::millisecond_t max_wait_time);
};

/*
SparkFLEX_interm

A class which interacts with the REV API to control and get information from
SparkFLEX hardware. Derives from SparkMXFX_interm.
*/
class SparkFLEX_interm : public SparkMXFX_interm {
public:
  SparkFLEX_interm(int can_id, units::millisecond_t max_wait_time);
};

}  // namespace frc846::control::hardware