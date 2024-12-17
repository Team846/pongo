#include "frc846/control/simulation/MCSimulator.h"

#include "frc846/control/calculators/CurrentTorqueCalculator.h"
#include "frc846/control/calculators/VelocityPositionEstimator.h"

namespace frc846::control::simulation {

MCSimulator::MCSimulator(frc846::control::base::MotorSpecs specs,
                         frc846::wpilib::unit_ohm circuit_resistance,
                         frc846::wpilib::unit_kg_m_sq rotational_inertia)
    : specs(specs),
      circuit_resistance_{circuit_resistance},
      rotational_inertia_{rotational_inertia} {
  last_tick_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch());
}

void MCSimulator::Tick(units::volt_t battery_voltage,
                       units::newton_meter_t load) {
  double duty_cycle = 0.0;
  if (auto* dc = std::get_if<double>(&control_message)) {
    duty_cycle = *dc;
  } else if (auto* vel =
                 std::get_if<units::radians_per_second_t>(&control_message)) {
    duty_cycle =
        gains.calculate(vel->to<double>(), 0.0, 0.0, load.to<double>());
  } else if (auto* pos = std::get_if<units::radian_t>(&control_message)) {
    duty_cycle = gains.calculate(pos->to<double>(), 0.0, velocity_.to<double>(),
                                 load.to<double>());
  }

  units::newton_meter_t torque_output =
      frc846::control::calculators::CurrentTorqueCalculator::predict_torque(
          duty_cycle, velocity_, battery_voltage, circuit_resistance_, specs);

  std::chrono::microseconds current_time =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  units::millisecond_t loop_time{(current_time - last_tick_).count() / 1000.0};
  last_tick_ = current_time;

  units::radians_per_second_t new_velocity =
      frc846::control::calculators::VelocityPositionEstimator::predict_velocity(
          velocity_, torque_output, loop_time, rotational_inertia_);
  units::radians_per_second_t avg_velocity = (velocity_ + new_velocity) / 2.0;

  position_ =
      frc846::control::calculators::VelocityPositionEstimator::predict_position(
          avg_velocity, position_, loop_time);

  velocity_ = new_velocity;
}

void MCSimulator::WriteDC(double duty_cycle) { control_message = duty_cycle; }
void MCSimulator::WriteVelocity(units::radians_per_second_t velocity) {
  control_message = velocity;
}
void MCSimulator::WritePosition(units::radian_t position) {
  control_message = position;
}

units::radian_t MCSimulator::GetPosition() { return position_; }
units::radians_per_second_t MCSimulator::GetVelocity() { return velocity_; }

void MCSimulator::ZeroEncoder(units::radian_t position) {
  this->position_ = position;
}

void MCSimulator::DisablePositionPacket() { position_packet_enabled = false; }
void MCSimulator::DisableVelocityPacket() { velocity_packet_enabled = false; }

void MCSimulator::SetGains(frc846::control::base::MotorGains gains) {
  this->gains = gains;
}

}  // namespace frc846::control::simulation