#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/torque.h>
#include <units/voltage.h>

#include <variant>

#include "frc846/control/base/motor_gains.h"
#include "frc846/control/base/motor_specs.h"

namespace frc846::control::simulation {

/*
MCSimulator

A class that simulates a motor controller. Uses motor dynamics to simulate
position and velocity accurately.
*/
class MCSimulator {
 public:
  MCSimulator(frc846::control::base::MotorSpecs specs);

  /*
  Tick()

  Uses the control message to update the motor state.
  */
  void Tick(units::volt_t battery_voltage);

  void WriteDC(double duty_cycle);
  void WriteTorque(units::newton_meter_t torque);
  void WriteVelocity(units::radians_per_second_t velocity);
  void WritePosition(units::radian_t position);

  units::radian_t GetPosition();
  units::radians_per_second_t GetVelocity();

  /*
  ZeroEncoder()

  Sets the motor's position to specified value.
  */
  void ZeroEncoder(units::radian_t position = 0_rad);

  /*
  DisablePositionPacket()

  Simulates disabling the status frame which contains the motor's position.
  */
  void DisablePositionPacket();
  /*
  DisableVelocityPacket()

  Simulates disabling the status frame which contains the motor's velocity.
  */
  void DisableVelocityPacket();

  void SetGains(frc846::control::base::MotorGains gains);

 private:
  frc846::control::base::MotorSpecs specs;
  frc846::control::base::MotorGains gains;

  bool velocity_packet_enabled = true;
  bool position_packet_enabled = true;

  units::radian_t position = 0_rad;
  units::radians_per_second_t velocity = 0_rad_per_s;

  std::variant<double, units::newton_meter_t, units::radians_per_second_t,
               units::radian_t>
      control_message = 0.0;
};

}  // namespace frc846::control::simulation