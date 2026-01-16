#include "subsystems/hardware/shooter.h"

ShooterSubsystem::ShooterSubsystem()
    : frc846::robot::GenericSubsystem<ShooterReadings,
          ShooterTarget>{"Shooter"},
      esc_(frc846::control::base::SPARK_MAX_VORTEX),
      esc_2_(frc846::control::base::SPARK_MAX_VORTEX) {
  esc_helper_.bind(&esc_);
  esc_helper_2_.bind(&esc_2_);

  RegisterPreference("kP", 0.01);
}

ShooterTarget ShooterSubsystem::ZeroTarget() const {
  return ShooterTarget{0.0};
}

bool ShooterSubsystem::VerifyHardware() { return true; }

void ShooterSubsystem::Setup() {
  esc_.Setup(frc846::control::config::MotorConstructionParameters{
      .can_id = 54,
      .inverted = false,
      .brake_mode = true,
      .motor_current_limit = 45_A,
      .smart_current_limit = 45_A,
      .voltage_compensation = 14_V,
      .circuit_resistance = unit_ohm(0),
      .rotational_inertia = unit_kg_m_sq(0),
      .friction = 0.02,
      .bus = "",
  });

  esc_2_.Setup(frc846::control::config::MotorConstructionParameters{
      .can_id = 55,
      .inverted = false,
      .brake_mode = true,
      .motor_current_limit = 45_A,
      .smart_current_limit = 45_A,
      .voltage_compensation = 14_V,
      .circuit_resistance = unit_ohm(0),
      .rotational_inertia = unit_kg_m_sq(0),
      .friction = 0.02,
      .bus = "",
  });

  esc_helper_.bind(&esc_);
  esc_helper_2_.bind(&esc_2_);

  esc_helper_.SetConversion(1_tr / 1.6_tr);
  esc_helper_2_.SetConversion(1_tr / 1.6_tr);
}

void ShooterSubsystem::WriteToHardware(ShooterTarget target) {
  // Get percent velocity of motor
  units::radians_per_second_t velocity = esc_.GetVelocity();
  units::radians_per_second_t velocity_2 = esc_2_.GetVelocity();
  units::dimensionless::scalar_t current_percent_velocity =
      (velocity + velocity_2) / 2.0 / (6826_rpm);

  double error = target.percent - current_percent_velocity.to<double>();
  double new_target = error * GetPreferenceValue_double("kP");

  esc_helper_.WriteDC(new_target);
  esc_helper_2_.WriteDC(new_target);
}

ShooterReadings ShooterSubsystem::ReadFromHardware() {
  return ShooterReadings{};
}