#include "frc846/control/hardware/simulation/MCSimulator.h"

#include <units/math.h>

#include <iostream>

#include "frc846/control/calculators/CurrentTorqueCalculator.h"
#include "frc846/control/calculators/VelocityPositionEstimator.h"

namespace frc846::control::simulation {

MCSimulator::MCSimulator(frc846::control::base::MotorSpecs specs,
    frc846::wpilib::unit_ohm circuit_resistance,
    frc846::wpilib::unit_kg_m_sq rotational_inertia, double friction)
    : specs(specs),
      circuit_resistance_{circuit_resistance},
      rotational_inertia_{rotational_inertia},
      friction_{friction} {
  last_tick_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch());
}

// TODO: cleanup iostream

void MCSimulator::Tick() {
  double duty_cycle = 0.0;
  if (auto* dc = std::get_if<double>(&control_message)) {
    duty_cycle = *dc;
  } else if (auto* vel =
                 std::get_if<units::radians_per_second_t>(&control_message)) {
    duty_cycle = gains.calculate((vel->to<double>() - velocity_.to<double>()),
        0.0, 0.0, vel->to<double>());
  } else if (auto* pos = std::get_if<units::radian_t>(&control_message)) {
    // std::cout << "Error: " << (pos->to<double>() - position_.to<double>())
    //           << std::endl;
    // std::cout << "Gains: " << gains.kP << ", " << gains.kI << ", " <<
    // gains.kD
    //           << ", " << gains.kFF << std::endl;
    duty_cycle = gains.calculate((pos->to<double>() - position_.to<double>()),
        0.0, velocity_.to<double>(), load_.to<double>());
  }
  duty_cycle = std::clamp(duty_cycle, -1.0, 1.0);

  // std::cout << "Duty Cycle: " << duty_cycle << std::endl;
  // std::cout << "Velocity: " << velocity_.to<double>()
  //           << " rad/s, Position: " << position_.to<double>() << " rad"
  //           << std::endl;

  pred_current_ = frc846::control::calculators::CurrentTorqueCalculator::
      predict_current_draw(
          duty_cycle, velocity_, battery_voltage_, circuit_resistance_, specs);

  pred_current_ = units::math::min(
      units::math::max(pred_current_, -current_limit_), current_limit_);

  // std::cout << "Predicted Current: " << pred_current_.to<double>() << " A"
  //           << std::endl;
  if (std::fabs(duty_cycle) < 0.03 && pred_current_ < 0_A && !brake_mode_)
    pred_current_ = 0_A;

  units::newton_meter_t friction_mag = specs.stall_torque * friction_;

  std::chrono::microseconds current_time =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  units::millisecond_t loop_time{(current_time - last_tick_).count() / 1000.0};
  last_tick_ = current_time;

  loop_time = units::math::min(loop_time, 30_ms);

  // std::cout << "Loop Time: " << loop_time.to<double>() << " ms" << std::endl;

  units::radians_per_second_t new_velocity =
      frc846::control::calculators::VelocityPositionEstimator::predict_velocity(
          velocity_, duty_cycle, loop_time, current_limit_, load_, friction_mag,
          rotational_inertia_, specs, brake_mode_);
  // new_velocity =
  //     units::radians_per_second_t{std::clamp(new_velocity.to<double>(),
  //         (-specs.free_speed).to<double>(), specs.free_speed.to<double>())};
  units::radians_per_second_t avg_velocity = (velocity_ + new_velocity) / 2.0;

  position_ =
      frc846::control::calculators::VelocityPositionEstimator::predict_position(
          avg_velocity, position_, loop_time);

  velocity_ = new_velocity;
}

void MCSimulator::SetInverted(bool inverted) {
  this->inverted = inverted;
  position_ = -position_;
}

void MCSimulator::WriteDC(double duty_cycle) {
  if (inverted) duty_cycle = -duty_cycle;
  control_message = duty_cycle;
}
void MCSimulator::WriteVelocity(units::radians_per_second_t velocity) {
  if (inverted) velocity = -velocity;
  control_message = velocity;
}
void MCSimulator::WritePosition(units::radian_t position) {
  if (inverted) position_ = -position_;
  control_message = position;
}

units::ampere_t MCSimulator::GetCurrent() {
  return units::math::abs(pred_current_);
}
units::radian_t MCSimulator::GetPosition() {
  return inverted ? -position_ : position_;
}
units::radians_per_second_t MCSimulator::GetVelocity() {
  return inverted ? -velocity_ : velocity_;
}

void MCSimulator::ZeroEncoder(units::radian_t position) {
  if (inverted) position = -position;
  this->position_ = position;
}

void MCSimulator::DisablePositionPacket() { position_packet_enabled = false; }
void MCSimulator::DisableVelocityPacket() { velocity_packet_enabled = false; }

void MCSimulator::EnableStatusFrames(
    std::vector<frc846::control::config::StatusFrame> frames,
    units::millisecond_t faults_ms, units::millisecond_t velocity_ms,
    units::millisecond_t encoder_position_ms,
    units::millisecond_t analog_position_ms) {
  bool disable_pos = true;
  bool disable_vel = true;
  for (auto frame : frames) {
    switch (frame) {
    case frc846::control::config::StatusFrame::kPositionFrame:
      disable_pos = false;
      break;
    case frc846::control::config::StatusFrame::kVelocityFrame:
      disable_vel = false;
      break;
    default: break;
    }
  }
  if (disable_pos) DisablePositionPacket();
  if (disable_vel) DisableVelocityPacket();
}

bool MCSimulator::IsDuplicateControlMessage(double duty_cycle) {
  if (inverted) duty_cycle = -duty_cycle;
  return std::holds_alternative<double>(control_message) &&
         std::get<double>(control_message) == duty_cycle;
}
bool MCSimulator::IsDuplicateControlMessage(
    units::radians_per_second_t velocity) {
  if (inverted) velocity = -velocity;
  return std::holds_alternative<units::radians_per_second_t>(control_message) &&
         std::get<units::radians_per_second_t>(control_message) == velocity;
}
bool MCSimulator::IsDuplicateControlMessage(units::radian_t position) {
  if (inverted) position = -position;
  return std::holds_alternative<units::radian_t>(control_message) &&
         std::get<units::radian_t>(control_message) == position;
}

void MCSimulator::SetGains(frc846::control::base::MotorGains gains) {
  this->gains = gains;
}

void MCSimulator::SetLoad(units::newton_meter_t load) {
  if (inverted) load = -load;
  this->load_ = load;
}
void MCSimulator::SetBatteryVoltage(units::volt_t voltage) {
  this->battery_voltage_ = voltage;
}

}  // namespace frc846::control::simulation