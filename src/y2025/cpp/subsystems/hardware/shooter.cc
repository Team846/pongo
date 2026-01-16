#include "subsystems/hardware/shooter.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/SubsystemHelper.h"
#include "subsystems/robot_constants.h"

ShooterSubsystem::ShooterSubsystem()
    : GenericSubsystem("shooter"),
      motor_configs_1_{.can_id = ports::proto_::kProtoMotor1_CANID,
          .inverted = false,
          .brake_mode = false,
          .motor_current_limit = 40_A,
          .smart_current_limit = 30_A,
          .voltage_compensation = 12_V,
          .circuit_resistance = robot_constants::climber_::wire_resistance,
          .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}},
        motor_configs_2_{.can_id = ports::proto_::kProtoMotor2_CANID,
          .inverted = true,
          .brake_mode = false,
          .motor_current_limit = 40_A,
          .smart_current_limit = 30_A,
          .voltage_compensation = 12_V,
          .circuit_resistance = robot_constants::climber_::wire_resistance,
          .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}},

      esc_1_{frc846::control::base::SPARK_MAX_VORTEX, motor_configs_1_},
      esc_2_{frc846::control::base::SPARK_MAX_VORTEX, motor_configs_2_} {
  esc_helper_1_.bind(&esc_1_);
  esc_helper_2_.bind(&esc_2_);
  esc_helper_1_.SetConversion(1_tr / 2_tr);
  esc_helper_2_.SetConversion(1_tr / 2_tr);
  REGISTER_SOFTLIMIT_CONFIG(false, 90_deg, 0_deg, 90_deg, 0_deg, 0.3);

  RegisterPreference("write_dc", 1.0);
}

ShooterTarget ShooterSubsystem::ZeroTarget() const { return {0.0}; }

void ShooterSubsystem::Setup() {
  esc_1_.Setup();
  esc_2_.Setup();

  esc_1_.EnableStatusFrames({
      frc846::control::config::StatusFrame::kPositionFrame,
      frc846::control::config::StatusFrame::kFaultFrame,
  });
  esc_2_.EnableStatusFrames({
      frc846::control::config::StatusFrame::kPositionFrame,
      frc846::control::config::StatusFrame::kFaultFrame,
  });

  esc_helper_1_.SetPosition(0_deg);
  esc_helper_2_.SetPosition(0_deg);

  esc_helper_1_.SetSoftLimits(GET_SOFTLIMITS(units::degree_t));
  esc_helper_2_.SetSoftLimits(GET_SOFTLIMITS(units::degree_t));
}

bool ShooterSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_1_.VerifyConnected(), ok, "Could not verify proto motor");
  FRC846_VERIFY(esc_2_.VerifyConnected(), ok, "Could not verify proto motor");
  return ok;
}

ShooterReadings ShooterSubsystem::ReadFromHardware() {}

void ShooterSubsystem::WriteToHardware(ShooterTarget target) {
  Graph("target/dc", target.duty_cycle_);

  esc_helper_1_.WriteDC(target.duty_cycle_);
  esc_helper_2_.WriteDC(target.duty_cycle_);
}