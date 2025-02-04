#include "subsystems/hardware/algal/algal_wrist.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/robot_constants.h"

AlgalWristSubsystem::AlgalWristSubsystem()
    : WristSubsystem("algal_wrist",
          frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO,
          frc846::control::config::MotorConstructionParameters{
              .can_id = ports::algal_ss_::wrist_::kWristMotor_CANID,
              .inverted = false,
              .brake_mode = true,
              .motor_current_limit = 40_A,
              .smart_current_limit =
                  30_A,  // TODO: prefify current limits (only)
              .voltage_compensation = 12_V,
              .circuit_resistance = robot_constants::algae_ss_::wire_resistance,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{3.0}

          },
          1.0_tr / 4.0_tr) {}

WristTarget AlgalWristSubsystem::ZeroTarget() const {
  return WristTarget{0_deg};
}

void AlgalWristSubsystem::ExtendedSetup() {
  // TODO: implement
}

std::pair<units::degree_t, bool> AlgalWristSubsystem::GetSensorPos() {
  // TODO: implement
  return {0_deg, false};
}