#include "subsystems/hardware/algal/algal_end_effector.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/robot_constants.h"

AlgalEESubsystem::AlgalEESubsystem()
    : GenericSubsystem("algal_end_effector"),
      motor_configs_{
          .can_id =
              ports::algal_ss_::end_effector_::kEE1_CANID,  // TODO: both ids
          .inverted = false,
          .brake_mode = true,
          .motor_current_limit = 40_A,
          .smart_current_limit = 30_A,  // TODO: prefify current limits (only)
          .voltage_compensation = 12_V,
          .circuit_resistance = robot_constants::algae_ss_::wire_resistance,
          .rotational_inertia = frc846::wpilib::unit_kg_m_sq{3.0},
      },
      esc_1_{frc846::control::base::SPARK_MAX_NEO550,
          motor_configs_},  // TODO: invert one of them
      esc_2_{frc846::control::base::SPARK_MAX_NEO550,
          getModifiedConfig(motor_configs_, true)} {}

frc846::control::config::MotorConstructionParameters
AlgalEESubsystem::getModifiedConfig(
    frc846::control::config::MotorConstructionParameters ogconfig,
    bool isOtherMotor) {
  frc846::control::config::MotorConstructionParameters modifiedConfig =
      ogconfig;

  if (isOtherMotor) {
    modifiedConfig.can_id = ports::algal_ss_::end_effector_::kEE2_CANID;
    modifiedConfig.inverted = !ogconfig.inverted;
  }

  return modifiedConfig;
}

void AlgalEESubsystem::Setup() {
  esc_1_.Setup();
  esc_2_.Setup();

  // TODO: finish
}

bool AlgalEESubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_1_.VerifyConnected(), ok, "Could not verify esc 1");
  FRC846_VERIFY(esc_2_.VerifyConnected(), ok, "Could not verify esc 2");
  return ok;
}

AlgalEEReadings AlgalEESubsystem::ReadFromHardware() {
  AlgalEEReadings readings;
  readings.has_piece_ = false;
  Graph("readings/has_piece", readings.has_piece_);
  return readings;
}

void AlgalEESubsystem::WriteToHardware(AlgalEETarget target) {
  Graph("target/duty_cycle", target.duty_cycle_);

  esc_1_.WriteDC(target.duty_cycle_);
  esc_2_.WriteDC(target.duty_cycle_);
}