#include "subsystems/hardware/generic/iclin.h"

#include "pdcsu.h"
#include "subsystems/SubsystemHelper.h"

IclinSubsystem::IclinSubsystem(std::string name,
    frc846::control::base::MotorMonkeyType mmtype,
    frc846::control::config::MotorConstructionParameters motor_configs_,
    linear_pos_conv_t conversion, units::inch_t hall_effect_loc_)
    : frc846::robot::GenericSubsystem<IclinReadings, IclinTarget>(name),
      linear_esc_(mmtype, GetCurrentConfig(motor_configs_)),
      bldc(105_u_A, 1.8_u_A, 2.6_u_Nm, 5676_u_rpm),
      lin_sys(bldc, 1, 214.85_u_rot / 262.5_u_in, 0.02_u_mps2, 5_u_kg, 22_u_N,
          3_u_N / 5676_u_rpm, 20_u_ms),
      icnor(lin_sys) {
  linear_esc_helper_.SetConversion(conversion);
  linear_esc_helper_.bind(&linear_esc_);
}

frc846::control::config::MotorConstructionParameters
IclinSubsystem::GetCurrentConfig(
    frc846::control::config::MotorConstructionParameters original_config) {
  frc846::control::config::MotorConstructionParameters modifiedConfig =
      original_config;
  REGISTER_MOTOR_CONFIG(
      original_config.motor_current_limit, original_config.smart_current_limit);
  modifiedConfig.motor_current_limit =
      GetPreferenceValue_unit_type<units::ampere_t>(
          "motor_configs/current_limit");
  modifiedConfig.smart_current_limit =
      GetPreferenceValue_unit_type<units::ampere_t>(
          "motor_configs/smart_current_limit");
  return modifiedConfig;
}

void IclinSubsystem::Setup() {
  linear_esc_.Setup();

  linear_esc_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame,
          frc846::control::config::StatusFrame::kFaultFrame});

  linear_esc_helper_.SetPosition(28.5_in);

  linear_esc_helper_.SetSoftLimits(GET_SOFTLIMITS(units::inch_t));

  icnor.setTolerance(lin_sys.toNative(0.25_u_in), lin_sys.toNative(0.4_u_in));
  icnor.setConstraints(
      5000_u_rpm, 1_u_A * GetPreferenceValue_unit_type<units::ampere_t>(
                              "motor_configs/smart_current_limit")
                              .to<double>());
  icnor.setProjectionHorizon(2);

  ExtendedSetup();
}

void IclinSubsystem::HomeSubsystem(units::inch_t pos) {
  if (is_initialized()) linear_esc_helper_.SetPosition(pos);
  is_homed_ = true;
}

bool IclinSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(linear_esc_.VerifyConnected(), ok, "Could not verify esc");
  return ok;
}

IclinReadings IclinSubsystem::ReadFromHardware() {
  IclinReadings readings;
  readings.position = linear_esc_helper_.GetPosition();

  linear_esc_.SetLoad(1_Nm);

  Graph("readings/position", readings.position);
  Graph("readings/current_draw", linear_esc_.GetCurrent());

  Graph("readings/error", GetTarget().position - readings.position);

  return readings;
}

void IclinSubsystem::OverrideSoftLimits(bool overrideLimits) {
  frc846::control::SoftLimitsConfig limits = GET_SOFTLIMITS(units::inch_t);

  limits.using_limits = !overrideLimits;
  linear_esc_helper_.SetSoftLimits(limits);
}

void IclinSubsystem::WriteToHardware(IclinTarget target) {
  Graph("target/position", target.position);
  linear_esc_.SetGains(GET_PIDF_GAINS());  // Doesn't really matter

  linear_esc_.SetLoad(1_Nm);  // Doesn't really matter

  radian_t tpos_as_native =
      lin_sys.toNative(1_u_in * target.position.to<double>());
  radps_t tvel_as_native = 0_u_radps;
  radian_t cpos_as_native =
      lin_sys.toNative(1_u_in * GetReadings().position.to<double>());
  radps_t cvel_as_native = lin_sys.toNative(
      1_u_in / 1_u_s * linear_esc_helper_.GetVelocity().to<double>());

  cpos_as_native +=
      cvel_as_native * 20_u_ms;  // TODO: better latency compensation

  linear_esc_helper_.WriteDC(icnor.getOutput(
      tpos_as_native, tvel_as_native, cpos_as_native, cvel_as_native));
}

void IclinSubsystem::BrakeSubsystem() {
  if (is_initialized()) linear_esc_.SetNeutralMode(true);
}

void IclinSubsystem::CoastSubsystem() {
  if (is_initialized()) linear_esc_.SetNeutralMode(false);
}
