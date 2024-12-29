#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/torque.h>
#include <units/voltage.h>

#include <variant>

#include "frc846/control/hardware/IntermediateController.h"
#include "frc846/control/base/motor_gains.h"
#include "frc846/control/base/motor_specs.h"
#include "frc846/wpilib/846_units.h"

namespace frc846::control::simulation {

/*
MCSimulator

A class that simulates a motor controller. Uses motor dynamics to simulate
position and velocity accurately.
*/
class MCSimulator : frc846::control::hardware::IntermediateController {
public:
  MCSimulator(frc846::control::base::MotorSpecs specs,
      frc846::wpilib::unit_ohm circuit_resistance,
      frc846::wpilib::unit_kg_m_sq rotational_inertia);

  /*
  Tick()

  Uses the control message to update the motor state.
  */
  void Tick(units::volt_t battery_voltage, units::newton_meter_t load);

  void WriteDC(double duty_cycle) override;
  void WriteVelocity(units::radians_per_second_t velocity) override;
  void WritePosition(units::radian_t position) override;

  units::radian_t GetPosition() override;
  units::radians_per_second_t GetVelocity() override;

  /*
  ZeroEncoder()

  Sets the motor's position to specified value.
  */
  void ZeroEncoder(units::radian_t position = 0_rad) override;

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

  void SetGains(frc846::control::base::MotorGains gains) override;

private:
  frc846::control::base::MotorSpecs specs;
  frc846::control::base::MotorGains gains;

  frc846::wpilib::unit_ohm circuit_resistance_{0};
  frc846::wpilib::unit_kg_m_sq rotational_inertia_{0};

  bool velocity_packet_enabled = true;
  bool position_packet_enabled = true;

  std::chrono::microseconds last_tick_;

  units::radian_t position_ = 0_rad;
  units::radians_per_second_t velocity_ = 0_rad_per_s;

  std::variant<double, units::radians_per_second_t, units::radian_t>
      control_message = 0.0;
};

}  // namespace frc846::control::simulation