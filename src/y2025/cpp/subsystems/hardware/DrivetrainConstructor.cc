#include "subsystems/hardware/DrivetrainConstructor.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "frc846/math/constants.h"
#include "ports.h"
#include "subsystems/robot_constants.h"

DrivetrainConstructor::DrivetrainConstructor()
    : Loggable{"DrivetrainConstructor"} {
  RegisterPreference("drive_motor_current_limit", 160_A);
  RegisterPreference("steer_motor_current_limit", 120_A);
  RegisterPreference("drive_motor_smart_current_limit", 120_A);
  RegisterPreference("steer_motor_smart_current_limit", 80_A);

  RegisterPreference("drive_motor_voltage_compensation", 16_V);
  RegisterPreference("steer_motor_voltage_compensation", 10_V);
}

frc846::robot::swerve::DrivetrainConfigs
DrivetrainConstructor::getDrivetrainConfigs() {
  frc846::control::base::MotorMonkeyType mmtype =
      frc846::control::base::MotorMonkeyType::TALON_FX_KRAKENX60;

  frc846::robot::swerve::DrivetrainConfigs configs;

  /*
  START SETTABLES
  TODO: Set these values during season
  */

  configs.navX_connection_mode = studica::AHRS::NavXComType::kMXP_SPI;

  units::inch_t wheel_diameter = 4_in;

  units::inch_t wire_length_FL = 15_in;
  units::inch_t wire_length_FR = 15_in;
  units::inch_t wire_length_BL = 35_in;
  units::inch_t wire_length_BR = 42_in;

  unsigned int num_connectors_FR = 0;
  unsigned int num_connectors_FL = 1;
  unsigned int num_connectors_BL = 2;
  unsigned int num_connectors_BR = 2;

  double drive_gear_ratio = 6.75;
  frc846::robot::swerve::drive_conv_unit drive_reduction =
      (frc846::math::constants::geometry::pi * wheel_diameter) /
      (drive_gear_ratio * 1_tr);
  frc846::robot::swerve::steer_conv_unit steer_reduction = 7_tr / 150_tr;

  configs.wheelbase_forward_dim = robot_constants::base::wheelbase_y;
  configs.wheelbase_horizontal_dim = robot_constants::base::wheelbase_x;

  units::pound_t wheel_approx_weight = 2_lb;
  units::inch_t wheel_weight_radius = 1_in;

  units::pound_t robot_weight = robot_constants::total_weight;

  // (Mass wheel) * (wheel_r)^2 * (steer reduction)^2
  frc846::wpilib::unit_kg_m_sq relative_steer_inertia{
      wheel_approx_weight * (wheel_weight_radius * wheel_weight_radius) *
      (steer_reduction * steer_reduction).to<double>()};

  // (Mass robot) * [(wheel_d)/2]^2 * (drive reduction)^2
  frc846::wpilib::unit_kg_m_sq relative_drive_inertia{
      robot_weight * (wheel_diameter * wheel_diameter) *
      (drive_reduction * drive_reduction).to<double>()};

  /* END SETTABLES */

  // Max accel = (4 * [Max force per wheel]) / (robot_weight)
  units::meter_t effective_torque_radius =
      (wheel_diameter / 2.0) / drive_gear_ratio;
  units::newton_t max_force_per_wheel{
      frc846::control::base::MotorSpecificationPresets::get(mmtype)
          .stall_torque /
      effective_torque_radius};
  configs.max_accel = (4.0 * max_force_per_wheel) / robot_weight;

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
          steer_params, mmtype, steer_reduction, drive_reduction, ""};
  configs.module_unique_configs = {FR_config, FL_config, BL_config, BR_config};

  configs.camera_x_offsets = {-5.875_in, 7_in};
  configs.camera_y_offsets = {4_in, -12_in};

  configs.cams = 2;
  configs.april_locations = {{1, {28.429_in, 31.594_in}},
      {2, {293.83_in, 35.41_in}}, {3, {317.15_in, 238.975_in}},
      {6, {128.5_in, 163.18_in}}, {7, {155.25_in, 144_in}},
      {8, {184.53_in, 158.1_in}}, {9, {189_in, 190.3_in}},
      {10, {161.75_in, 209.5_in}}, {11, {131.7_in, 195.9_in}},
      {12, {23.17_in, 655.45_in}}, {13, {289.3_in, 660_in}},
      {16, {-0.15_in, 451.895_in}}, {17, {127.87_in, 528.19_in}},
      {18, {161.75_in, 546.875_in}}, {19, {189.12_in, 532.68_in}},
      {20, {184.53_in, 495.48_in}}, {21, {155.25_in, 481.375_in}},
      {22, {132.47_in, 500.07_in}}};

  return configs;
}