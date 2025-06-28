#include "frc846/robot/swerve/swerve_module.h"

#include <units/math.h>

#include <thread>

#include "frc/RobotBase.h"
#include "frc846/math/collection.h"

namespace frc846::robot::swerve {

SwerveModuleSubsystem::SwerveModuleSubsystem(Loggable& parent,
    SwerveModuleUniqueConfig unique_config,
    SwerveModuleCommonConfig common_config)
    : frc846::robot::GenericSubsystem<SwerveModuleReadings, SwerveModuleTarget>(
          parent, unique_config.loc),
      drive_{common_config.motor_types,
          getMotorParams(unique_config, common_config).first},
      steer_{common_config.motor_types,
          getMotorParams(unique_config, common_config).second},
      cancoder_{unique_config.cancoder_id, common_config.bus},
      steer_load_factor_{common_config.steer_load_factor} {
  drive_helper_.SetConversion(common_config.drive_reduction);
  steer_helper_.SetConversion(common_config.steer_reduction);

  drive_helper_.bind(&drive_);
  steer_helper_.bind(&steer_);

  cancoder_.OptimizeBusUtilization();
  cancoder_.GetAbsolutePosition().SetUpdateFrequency(20_Hz);

  RegisterPreference("cancoder_offset_", 0.0_deg);

  max_speed_ = frc846::control::base::MotorSpecificationPresets::get(
                   common_config.motor_types)
                   .free_speed *
               common_config.drive_reduction;
}

std::pair<frc846::control::config::MotorConstructionParameters,
    frc846::control::config::MotorConstructionParameters>
SwerveModuleSubsystem::getMotorParams(SwerveModuleUniqueConfig unique_config,
    SwerveModuleCommonConfig common_config) {
  frc846::control::config::MotorConstructionParameters drive_params =
      common_config.drive_params;
  frc846::control::config::MotorConstructionParameters steer_params =
      common_config.steer_params;

  drive_params.can_id = unique_config.drive_id;
  steer_params.can_id = unique_config.steer_id;

  drive_params.bus = common_config.bus;
  steer_params.bus = common_config.bus;

  drive_params.circuit_resistance = unique_config.circuit_resistance;
  steer_params.circuit_resistance = unique_config.circuit_resistance;

  return {drive_params, steer_params};
}

void SwerveModuleSubsystem::Setup() {
  drive_.Setup();
  drive_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame},
      20_ms, 5_ms, 5_ms, 20_ms);
  drive_helper_.SetPosition(0_ft);

  steer_.Setup();
  steer_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame},
      20_ms, 20_ms, 5_ms, 20_ms);

  ZeroWithCANcoder();
}

SwerveModuleTarget SwerveModuleSubsystem::ZeroTarget() const {
  return SwerveModuleOLControlTarget{0.0_fps, 0_deg};
}

bool SwerveModuleSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(drive_.VerifyConnected(), ok, "Could not verify drive motor");
  FRC846_VERIFY(steer_.VerifyConnected(), ok, "Could not verify steer motor");
  return ok;
}

void SwerveModuleSubsystem::SetCANCoderOffset() {
  SetCANCoderOffset(cancoder_.GetAbsolutePosition().GetValue());
}
void SwerveModuleSubsystem::SetCANCoderOffset(units::degree_t offset) {
  SetPreferenceValue("cancoder_offset_", offset);
}

void SwerveModuleSubsystem::ZeroWithCANcoder() {
  if (frc::RobotBase::IsSimulation()) {
    steer_helper_.SetPosition(0_deg);
    return;
  }

  constexpr int kMaxAttempts = 5;
  constexpr int kSleepTimeMs = 500;

  last_rezero = 0;

  for (int attempts = 1; attempts <= kMaxAttempts; ++attempts) {
    Log("CANCoder zero attempt {}/{}", attempts, kMaxAttempts);
    auto position = cancoder_.GetAbsolutePosition();

    units::degree_t position_zero =
        -position.GetValue() +
        GetPreferenceValue_unit_type<units::degree_t>("cancoder_offset_");

    if (position.IsAllGood()) {
      steer_helper_.SetPosition(position_zero);
      Log("Zeroed to {}!", position_zero);
      return;
    } else if (attempts == kMaxAttempts) {
      Error("Unable to zero normally after {} attempts - attempting anyways",
          kMaxAttempts);
      steer_helper_.SetPosition(position_zero);
      Warn("Unreliably zeroed to {}!", position_zero);
      return;
    }

    Warn("Attempt to zero failed, sleeping {} ms...", kSleepTimeMs);

    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
  }
}

SwerveModuleReadings SwerveModuleSubsystem::ReadFromHardware() {
  SwerveModuleReadings readings;
  readings.vel = drive_helper_.GetVelocity();
  readings.drive_pos = drive_helper_.GetPosition();
  readings.steer_pos = steer_helper_.GetPosition();

  units::newton_meter_t pred_steer_load =
      steer_load_factor_ * readings.vel *
      (steer_helper_.GetVelocity().convert<units::radians_per_second>() /
          1_rad);

  steer_.SetLoad(pred_steer_load);

  Graph("readings/pred_steer_load", pred_steer_load);

  Graph("readings/drive_motor_vel", readings.vel);
  // Graph("readings/drive_motor_pos", readings.drive_pos);
  Graph("readings/steer_motor_pos", readings.steer_pos);

  Graph("readings/cancoder_pos",
      units::degree_t(cancoder_.GetAbsolutePosition().GetValue()));

  return readings;
}

void SwerveModuleSubsystem::WriteToHardware(SwerveModuleTarget target) {
  // Graph("target/drive_target", target.drive);
  // Graph("target/steer_target", target.steer);

  auto [steer_dir, invert_drive] =
      calculateSteerPosition(target.steer, GetReadings().steer_pos);

  Graph("target/steer_dir", steer_dir);
  // Graph("target/invert_drive", invert_drive);

  units::dimensionless::scalar_t cosine_comp =
      units::math::cos(target.steer - GetReadings().steer_pos);

  // Graph("target/cosine_comp", cosine_comp.to<double>());

  double drive_duty_cycle = cosine_comp * target.drive / max_speed_;

  Graph("target/drive_dc", drive_duty_cycle);

  drive_helper_.WriteDC(drive_duty_cycle);

  if (std::abs(drive_duty_cycle) > 0.002 || last_rezero < 50) {
    steer_helper_.WritePositionOnController(steer_dir);
    last_rezero += 1;
  }
}

std::pair<units::degree_t, bool> SwerveModuleSubsystem::calculateSteerPosition(
    units::degree_t target_norm, units::degree_t current) {
  bool invert = false;

  units::degree_t target = target_norm;

  while ((target - current) > 90_deg) {
    target -= 180_deg;
    invert = !invert;
  }
  while ((target - current) < -90_deg) {
    target += 180_deg;
    invert = !invert;
  }

  return {target, invert};
}

void SwerveModuleSubsystem::SetSteerGains(
    frc846::control::base::MotorGains gains) {
  steer_.SetGains(gains);
}

}  // namespace frc846::robot::swerve