#include "frc846/control/simulation/MCSimulator.h"

#include "frc846/control/calculators/CurrentTorqueCalculator.h"
#include "frc846/control/calculators/VelocityPositionEstimator.h"

namespace frc846::control::simulation {

MCSimulator::MCSimulator(frc846::control::base::MotorSpecs specs)
    : specs(specs) {}

void MCSimulator::Tick(units::volt_t battery_voltage) {
  units::newton_meter_t torque_output = 0_Nm;
  if (auto* torque = std::get_if<units::newton_meter_t>(&control_message)) {
    torque_output = *torque;
  } else {
    double duty_cycle = 0.0;
    if (auto* dc = std::get_if<double>(&control_message)) {
      duty_cycle = *dc;
    } else if (auto* velocity =
                   std::get_if<units::radians_per_second_t>(&control_message)) {
      double duty_cycle =
          gains.calculate(velocity->to<double>(), 0.0, 0.0, 0.0);
    }
  }
  //   frc846::control::calculators::CurrentTorqueCalculator::predict_torque(
  //       std::get<units::newton_meter_t>(control_message), velocity, 12_V,
  //       0_Ohm, specs);
}

void MCSimulator::WriteDC(double duty_cycle) { control_message = duty_cycle; }
void MCSimulator::WriteTorque(units::newton_meter_t torque) {
  control_message = torque;
}
void MCSimulator::WriteVelocity(units::radians_per_second_t velocity) {
  control_message = velocity;
}
void MCSimulator::WritePosition(units::radian_t position) {
  control_message = position;
}

units::radian_t MCSimulator::GetPosition() { return position; }
units::radians_per_second_t MCSimulator::GetVelocity() { return velocity; }

void MCSimulator::ZeroEncoder(units::radian_t position) {
  this->position = position;
}

void MCSimulator::DisablePositionPacket() { position_packet_enabled = false; }
void MCSimulator::DisableVelocityPacket() { velocity_packet_enabled = false; }

void MCSimulator::SetGains(frc846::control::base::MotorGains gains) {
  this->gains = gains;
}

}  // namespace frc846::control::simulation