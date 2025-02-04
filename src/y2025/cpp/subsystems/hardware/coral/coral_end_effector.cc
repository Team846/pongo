#include "subsystems/hardware/coral/coral_end_effector.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/robot_constants.h"

CoralEESubsystem::CoralEESubsystem()
    : GenericSubsystem("Coral_end_effector"),
      motor_configs_{
          .can_id = ports::coral_ss_::end_effector_::kEE_CANID,
          .inverted = false,
          .brake_mode = true,
          .motor_current_limit = 40_A,
          .smart_current_limit = 30_A,  // TODO: prefify current limits (only)
          .voltage_compensation = 12_V,
          .circuit_resistance = robot_constants::algae_ss_::wire_resistance,
          .rotational_inertia = frc846::wpilib::unit_kg_m_sq{3.0},
      },
      esc_{frc846::control::base::SPARK_MAX_NEO550, motor_configs_} {}

void CoralEESubsystem::Setup() {
  esc_.Setup();

  // TODO: finish
}

bool CoralEESubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_.VerifyConnected(), ok, "Could not verify esc");
  return ok;
}

CoralEEReadings CoralEESubsystem::ReadFromHardware() {
  CoralEEReadings readings;
  readings.has_piece_ = false;
  Graph("readings/has_piece", readings.has_piece_);
  return readings;
}

void CoralEESubsystem::WriteToHardware(CoralEETarget target) {
  Graph("target/duty_cycle", target.duty_cycle_);
  esc_.WriteDC(target.duty_cycle_);
}