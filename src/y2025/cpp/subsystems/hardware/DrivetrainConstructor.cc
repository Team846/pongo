#include "subsystems/hardware/DrivetrainConstructor.h"

frc846::robot::swerve::DrivetrainConfigs
DrivetrainConstructor::getDrivetrainConfigs() {
  frc846::robot::swerve::DrivetrainConfigs configs;

  /* START SETTABLES */
  // TODO: Set these values to the correct values for the robot in 2025
  configs.navX_connection_mode =
      frc846::robot::swerve::NavX_connection_type::kSerial;

  frc846::robot::swerve::steer_conv_unit steer_reduction = 1.0;  // TODO: fix
  frc846::robot::swerve::drive_conv_unit drive_reduction =
      1.0_ft / 1.0_tr;  // TODO: fix

  frc846::robot::swerve::SwerveModuleUniqueConfig FR_config{
      "FR", 999, 999, 999, 999_Ohm};
  frc846::robot::swerve::SwerveModuleUniqueConfig FL_config{
      "FL", 999, 999, 999, 999_Ohm};
  frc846::robot::swerve::SwerveModuleUniqueConfig BR_config{
      "BR", 999, 999, 999, 999_Ohm};
  frc846::robot::swerve::SwerveModuleUniqueConfig BL_config{
      "BL", 999, 999, 999, 999_Ohm};

  configs.wheelbase_forward_dim = 26_in;
  configs.wheelbase_horizontal_dim = 26_in;

  /* END SETTABLES */

  frc846::control::config::MotorConstructionParameters drive_params{
      .can_id = 999,
      .inverted = false,
      .brake_mode = true,
      .motor_current_limit = 999_A,
      .smart_current_limit = 999_A,
      .voltage_compensation = 999_V,
      .circuit_resistance = 999_Ohm,
      .rotational_inertia = frc846::wpilib::unit_kg_m_sq{999},
      .bus = "",
  };
  frc846::control::config::MotorConstructionParameters steer_params{
      .can_id = 999,
      .inverted = false,
      .brake_mode = false,
      .motor_current_limit = 999_A,
      .smart_current_limit = 999_A,
      .voltage_compensation = 999_V,
      .circuit_resistance = 999_Ohm,
      .rotational_inertia = frc846::wpilib::unit_kg_m_sq{999},
      .bus = "",
  };

  configs.module_common_config =
      frc846::robot::swerve::SwerveModuleCommonConfig{drive_params,
          steer_params,
          frc846::control::base::MotorMonkeyType::TALON_FX_KRAKENX60,
          steer_reduction, drive_reduction, ""};
  configs.module_unique_configs = {FR_config, FL_config, BR_config, BL_config};

  return configs;
}