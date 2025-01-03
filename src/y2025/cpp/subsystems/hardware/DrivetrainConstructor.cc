#include "subsystems/hardware/DrivetrainConstructor.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "frc846/math/constants.h"
#include "ports.h"

DrivetrainConstructor::DrivetrainConstructor()
    : Loggable{"DrivetrainConstructor"} {
  RegisterPreference("drive_motor_current_limit", 120_A);
  RegisterPreference("steer_motor_current_limit", 120_A);
  RegisterPreference("drive_motor_smart_current_limit", 80_A);
  RegisterPreference("steer_motor_smart_current_limit", 80_A);

  RegisterPreference("drive_motor_voltage_compensation", 16_V);
  RegisterPreference("steer_motor_voltage_compensation", 12_V);
}

frc846::robot::swerve::DrivetrainConfigs
DrivetrainConstructor::getDrivetrainConfigs() {
  frc846::robot::swerve::DrivetrainConfigs configs;

  /*
  START SETTABLES
  TODO: Set these values during season
  */

  configs.navX_connection_mode =
      frc846::robot::swerve::NavX_connection_type::kSerial;

  units::inch_t wheel_diameter = 4_in;

  units::inch_t wire_length_FL = 14_in;
  units::inch_t wire_length_FR = 14_in;
  units::inch_t wire_length_BL = 14_in;
  units::inch_t wire_length_BR = 14_in;

  unsigned int num_connectors_FR = 3;
  unsigned int num_connectors_FL = 3;
  unsigned int num_connectors_BL = 3;
  unsigned int num_connectors_BR = 3;

  frc846::robot::swerve::drive_conv_unit drive_reduction =
      (frc846::math::constants::geometry::pi * wheel_diameter) / 6.12_tr;
  frc846::robot::swerve::steer_conv_unit steer_reduction = 7_tr / 150_tr;

  configs.wheelbase_forward_dim = 26_in;
  configs.wheelbase_horizontal_dim = 26_in;

  units::pound_t wheel_approx_weight = 2_lb;
  units::inch_t wheel_weight_radius = 1_in;

  units::pound_t robot_weight = 120_lb;

  // (Mass wheel) * (wheel_r)^2 * (steer reduction)^2
  frc846::wpilib::unit_kg_m_sq relative_steer_inertia{
      wheel_approx_weight * (wheel_weight_radius * wheel_weight_radius) *
      (steer_reduction * steer_reduction).to<double>()};

  // (Mass robot) * [(wheel_d)/2]^2 * (drive reduction)^2
  frc846::wpilib::unit_kg_m_sq relative_drive_inertia{
      robot_weight * (wheel_diameter * wheel_diameter) *
      (drive_reduction * drive_reduction).to<double>()};

  /* END SETTABLES */

  frc846::wpilib::unit_ohm wire_resistance_FR{
      frc846::control::calculators::CircuitResistanceCalculator::calculate(
          wire_length_FR, frc846::control::calculators::fourteen_gauge,
          num_connectors_FR)};
  frc846::wpilib::unit_ohm wire_resistance_FL{
      frc846::control::calculators::CircuitResistanceCalculator::calculate(
          wire_length_FL, frc846::control::calculators::fourteen_gauge,
          num_connectors_FL)};

  frc846::wpilib::unit_ohm wire_resistance_BL{
      frc846::control::calculators::CircuitResistanceCalculator::calculate(
          wire_length_BL, frc846::control::calculators::fourteen_gauge,
          num_connectors_BL)};
  frc846::wpilib::unit_ohm wire_resistance_BR{
      frc846::control::calculators::CircuitResistanceCalculator::calculate(
          wire_length_BR, frc846::control::calculators::fourteen_gauge,
          num_connectors_BR)};

  frc846::robot::swerve::SwerveModuleUniqueConfig FR_config{"FR",
      ports::drivetrain_::kFRCANCoder_CANID, ports::drivetrain_::kFRDrive_CANID,
      ports::drivetrain_::kFRSteer_CANID, wire_resistance_FR};
  frc846::robot::swerve::SwerveModuleUniqueConfig FL_config{"FL",
      ports::drivetrain_::kFLCANCoder_CANID, ports::drivetrain_::kFLDrive_CANID,
      ports::drivetrain_::kFLSteer_CANID, wire_resistance_FL};
  frc846::robot::swerve::SwerveModuleUniqueConfig BL_config{"BL",
      ports::drivetrain_::kBLCANCoder_CANID, ports::drivetrain_::kBLDrive_CANID,
      ports::drivetrain_::kBLSteer_CANID, wire_resistance_BL};
  frc846::robot::swerve::SwerveModuleUniqueConfig BR_config{"BR",
      ports::drivetrain_::kBRCANCoder_CANID, ports::drivetrain_::kBRDrive_CANID,
      ports::drivetrain_::kBRSteer_CANID, wire_resistance_BR};

  frc846::control::config::MotorConstructionParameters drive_params{
      .can_id = 999,  // overriden by unique config
      .inverted = false,
      .brake_mode = true,
      .motor_current_limit = GetPreferenceValue_unit_type<units::ampere_t>(
          "drive_motor_current_limit"),
      .smart_current_limit = GetPreferenceValue_unit_type<units::ampere_t>(
          "drive_motor_smart_current_limit"),
      .voltage_compensation = GetPreferenceValue_unit_type<units::volt_t>(
          "drive_motor_voltage_compensation"),
      .circuit_resistance = 999_Ohm,  // overriden by unique config
      .rotational_inertia = relative_drive_inertia,
      .bus = "",
  };
  frc846::control::config::MotorConstructionParameters steer_params{
      .can_id = 999,  // overriden by unique config
      .inverted = false,
      .brake_mode = false,
      .motor_current_limit = GetPreferenceValue_unit_type<units::ampere_t>(
          "steer_motor_current_limit"),
      .smart_current_limit = GetPreferenceValue_unit_type<units::ampere_t>(
          "steer_motor_smart_current_limit"),
      .voltage_compensation = GetPreferenceValue_unit_type<units::volt_t>(
          "steer_motor_voltage_compensation"),
      .circuit_resistance = 999_Ohm,  // overriden by unique config
      .rotational_inertia = relative_steer_inertia,
      .bus = "",
  };

  configs.module_common_config =
      frc846::robot::swerve::SwerveModuleCommonConfig{drive_params,
          steer_params,
          frc846::control::base::MotorMonkeyType::TALON_FX_KRAKENX60,
          steer_reduction, drive_reduction, ""};
  configs.module_unique_configs = {FR_config, FL_config, BL_config, BR_config};

  return configs;
}