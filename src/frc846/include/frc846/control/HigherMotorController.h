#pragma once

#include <units/torque.h>

#include <optional>

#include "frc846/control/base/motor_control_base.h"
#include "frc846/control/base/motor_gains.h"
#include "frc846/control/base/motor_specs.h"
#include "frc846/control/config/construction_params.h"
#include "frc846/control/config/status_frames.h"

#include "frc846/control/config/soft_limits.h"

#include <optional>
#include <vector>

namespace frc846::control {

/*
HigherMotorController

A class that interfaces with MotorMonkey to provide higher-level control of
motors.
*/
class HigherMotorController {
public:
  HigherMotorController(frc846::control::base::MotorMonkeyType mmtype,
      frc846::control::config::MotorConstructionParameters params);

  // Sets up the motor. Gets a slot ID from MotorMonkey.
  void Setup();

  void SetGains(frc846::control::base::MotorGains gains);

  /*
  SetLoad()

  Sets the value of the load on the motor controller that is used in
  calculations. If this isn't called, previous value is maintained.
  */
  void SetLoad(units::newton_meter_t load);

  void WriteDC(double duty_cycle);
  void WriteCurrent(units::ampere_t current);
  void WriteTorque(units::newton_meter_t torque);

  /*
  WriteVelocity()

  Writes a velocity setpoint to the motor controller. PID calculations performed
  locally. If load is set, it is used to dynamically calculate the feedforward
  term.
  */
  void WriteVelocity(units::radians_per_second_t velocity);
  /*
  WritePosition()

  Writes a position setpoint to the motor controller. PID calculations performed
  locally. If load is set, it is used to dynamically calculate the feedforward
  term.
  */
  void WritePosition(units::radian_t position);
  /*
  WriteVelocityOnController()

  Writes a velocity setpoint to the motor controller. PID calculations performed
  onboard the motor controller.
  */
  void WriteVelocityOnController(units::radians_per_second_t velocity);
  /*
  WritePositionOnController()

  Writes a position setpoint to the motor controller. PID calculations performed
  onboard the motor controller.
  */
  void WritePositionOnController(units::radian_t position);

  units::radians_per_second_t GetVelocity();
  units::radian_t GetPosition();
  units::ampere_t GetCurrent();

  // Zeroes the encoder to the specified value
  void SetPosition(units::radian_t position);

  // Soft limits maintained by the motor controller
  void SetControllerSoftLimits(
      units::radian_t forward_limit, units::radian_t reverse_limit);

  // Custom soft limits maintained by HigherMotorController
  void SetSoftLimits(config::SoftLimits soft_limits);

  // Enables specific status frames and disables all others
  void EnableStatusFrames(std::vector<config::StatusFrame> frames);

private:
  frc846::control::base::MotorMonkeyType mmtype_;
  frc846::control::config::MotorConstructionParameters constr_params_;

  frc846::control::base::MotorGains gains_;

  units::newton_meter_t load_;

  std::optional<config::SoftLimits> soft_limits_;

  size_t slot_id_ = 999;

  bool called_enable_status_frames_ = false;
};

}  // namespace frc846::control