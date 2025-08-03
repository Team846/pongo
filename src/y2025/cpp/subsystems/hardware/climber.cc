#include "subsystems/hardware/climber.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/SubsystemHelper.h"
#include "subsystems/robot_constants.h"

ClimberSubsystem::ClimberSubsystem()
    : GenericSubsystem("climber"),
      motor_configs_{.can_id = ports::climber_::kClimber_CANID,
          .inverted = false,
          .brake_mode = true,
          .motor_current_limit = 40_A,
          .smart_current_limit = 30_A,
          .voltage_compensation = 12_V,
          .circuit_resistance = robot_constants::climber_::wire_resistance,
          .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}},
      esc_{frc846::control::base::SPARK_MAX_NEO, motor_configs_} {
  esc_helper_.bind(&esc_);
  esc_helper_.SetConversion(1_tr / 225_tr);
  REGISTER_SOFTLIMIT_CONFIG(true, 90_deg, 0_deg, 90_deg, 0_deg, 0.3);

  RegisterPreference("extend_dc", 1.0);
  RegisterPreference("retract_dc", -1.0);
}

ClimberTarget ClimberSubsystem::ZeroTarget() const { return {0.0}; }

void ClimberSubsystem::Setup() {
  esc_.Setup();

  esc_.EnableStatusFrames({
      frc846::control::config::StatusFrame::kPositionFrame,
      frc846::control::config::StatusFrame::kFaultFrame,
  });

  esc_helper_.SetPosition(0_deg);

  esc_helper_.SetSoftLimits(GET_SOFTLIMITS(units::degree_t));
}

bool ClimberSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_.VerifyConnected(), ok, "Could not verify climber motor");
  return ok;
}

ClimberReadings ClimberSubsystem::ReadFromHardware() {
  Graph("readings/position", esc_helper_.GetPosition());

  return {};
}

void ClimberSubsystem::WriteToHardware(ClimberTarget target) {
  Graph("target/dc", target.duty_cycle_);

  esc_helper_.WriteDC(target.duty_cycle_);
}

void ClimberSubsystem::BrakeSubsystem() {
  if (is_initialized()) esc_.SetNeutralMode(true);
}

void ClimberSubsystem::CoastSubsystem() {
  if (is_initialized()) esc_.SetNeutralMode(false);
}

void ClimberSubsystem::ZeroClimber() {
  if (is_initialized()) esc_helper_.SetPosition(0_deg);
}