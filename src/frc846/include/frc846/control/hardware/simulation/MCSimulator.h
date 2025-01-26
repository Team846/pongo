#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/torque.h>
#include <units/voltage.h>

#include <variant>

#include "frc846/control/base/motor_gains.h"
#include "frc846/control/base/motor_specs.h"
#include "frc846/control/hardware/IntermediateController.h"
#include "frc846/wpilib/units.h"

namespace frc846::control::simulation {

/*
MCSimulator

A class that simulates a motor controller. Uses motor dynamics to simulate
position and velocity accurately.
*/
class MCSimulator : public frc846::control::hardware::IntermediateController {
public:
  MCSimulator(frc846::control::base::MotorSpecs specs,
      frc846::wpilib::unit_ohm circuit_resistance,
      frc846::wpilib::unit_kg_m_sq rotational_inertia);

  /*
  Tick()

  Uses the control message to update the motor state.
  */
  void Tick() override;

  bool VerifyConnected() override { return true; }

  frc846::control::hardware::ControllerErrorCodes GetLastErrorCode() override {
    return frc846::control::hardware::ControllerErrorCodes::kAllOK;
  }

  void SetInverted(bool inverted) override;
  void SetNeutralMode(bool brake_mode) override { (void)brake_mode; };
  void SetCurrentLimit(units::ampere_t current_limit) { (void)current_limit; };

  void SetSoftLimits(
      units::radian_t forward_limit, units::radian_t reverse_limit) override {
    (void)forward_limit;
    (void)reverse_limit;
  };

  void SetVoltageCompensation(units::volt_t voltage_compensation) {
    (void)voltage_compensation;
  };

  void WriteDC(double duty_cycle) override;
  void WriteVelocity(units::radians_per_second_t velocity) override;
  void WritePosition(units::radian_t position) override;

  units::radian_t GetPosition() override;
  units::radians_per_second_t GetVelocity() override;
  units::ampere_t GetCurrent() override;

  virtual void ConfigForwardLimitSwitch(bool stop_motor,
      frc846::control::base::LimitSwitchDefaultState type) override {}

  virtual void ConfigReverseLimitSwitch(bool stop_motor,
      frc846::control::base::LimitSwitchDefaultState type) override {}

  virtual bool GetForwardLimitSwitchState() override { return false; }
  virtual bool GetReverseLimitSwitchState() override { return false; }

  virtual units::volt_t GetAnalogDeviceOutput() override { return 0.0_V; }

  /*
  ZeroEncoder()

  Sets the motor's position to specified value.
  */
  void ZeroEncoder(units::radian_t position) override;

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

  void EnableStatusFrames(
      std::vector<frc846::control::config::StatusFrame> frames) override;

  bool IsDuplicateControlMessage(double duty_cycle) override;
  bool IsDuplicateControlMessage(units::radians_per_second_t velocity) override;
  bool IsDuplicateControlMessage(units::radian_t position) override;

  void SetGains(frc846::control::base::MotorGains gains) override;

  void SetLoad(units::newton_meter_t load);
  void SetBatteryVoltage(units::volt_t voltage);

private:
  frc846::control::base::MotorSpecs specs;
  frc846::control::base::MotorGains gains;

  frc846::wpilib::unit_ohm circuit_resistance_{0};
  frc846::wpilib::unit_kg_m_sq rotational_inertia_{0};

  bool velocity_packet_enabled = true;
  bool position_packet_enabled = true;

  std::chrono::microseconds last_tick_;

  units::ampere_t pred_current_{0};
  units::radian_t position_ = 0_rad;
  units::radians_per_second_t velocity_ = 0_rad_per_s;

  std::variant<double, units::radians_per_second_t, units::radian_t>
      control_message = 0.0;

  units::newton_meter_t load_{0};
  units::volt_t battery_voltage_{0};

  bool inverted = false;
};

}  // namespace frc846::control::simulation