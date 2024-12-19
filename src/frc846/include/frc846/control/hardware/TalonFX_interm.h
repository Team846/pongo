#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <variant>

#include "IntermediateController.h"

namespace frc846::control::hardware {

class TalonFX_interm : public IntermediateController {
 public:
  TalonFX_interm(int can_id, std::string bus = "rio",
                 units::millisecond_t max_wait_time = 20_ms);

  void Tick() override;

  void SetInverted(bool inverted) override;
  void SetNeutralMode(bool brake_mode) override;
  void SetCurrentLimit(units::ampere_t current_limit) override;
  void SetSoftLimits(units::radian_t forward_limit,
                     units::radian_t reverse_limit) override;

  void SetVoltageCompensation(units::volt_t voltage_compensation) override;

  void SetGains(frc846::control::base::MotorGains gains) override;

  void WriteDC(double duty_cycle) override;
  void WriteVelocity(units::radians_per_second_t velocity) override;
  void WritePosition(units::radian_t position) override;

  void EnableStatusFrames(
      std::vector<frc846::control::config::StatusFrame> frames) override;

  bool IsDuplicateControlMessage(double duty_cycle) override;
  bool IsDuplicateControlMessage(units::radians_per_second_t velocity) override;
  bool IsDuplicateControlMessage(units::radian_t position) override;

  void ZeroEncoder(units::radian_t position) override;

  units::radians_per_second_t GetVelocity() override;
  units::radian_t GetPosition() override;
  units::ampere_t GetCurrent() override;

  ControllerErrorCodes GetLastErrorCode() override;

 private:
  frc846::control::hardware::ControllerErrorCodes getErrorCode(
      ctre::phoenix::StatusCode code);

  std::variant<units::radian_t, units::radians_per_second_t, double>
      last_command_;
  frc846::control::base::MotorGains gains_;

  ctre::phoenix6::hardware::TalonFX talon_;

  frc846::control::hardware::ControllerErrorCodes last_error_;

  units::millisecond_t max_wait_time_;
};

}  // namespace frc846::control::hardware