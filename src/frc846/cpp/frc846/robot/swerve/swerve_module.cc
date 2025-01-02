#include "frc846/robot/swerve/swerve_module.h"

#include <units/math.h>

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
      cancoder_{unique_config.cancoder_id, common_config.bus} {
  drive_helper_.SetConversion(common_config.drive_reduction);
  steer_helper_.SetConversion(common_config.steer_reduction);

  drive_helper_.bind(&drive_);
  steer_helper_.bind(&steer_);

  cancoder_.OptimizeBusUtilization();
  cancoder_.GetAbsolutePosition().SetUpdateFrequency(20_Hz);

  RegisterPreference("cancoder_offset_", 0.0_deg);
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
          frc846::control::config::StatusFrame::kVelocityFrame});
  drive_helper_.SetPosition(0_ft);

  steer_.Setup();
  steer_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame});

  ZeroWithCANcoder();
}

SwerveModuleTarget SwerveModuleSubsystem::ZeroTarget() const {
  return SwerveModuleOLControlTarget{0.0, 0_deg};
}

bool SwerveModuleSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(drive_.VerifyConnected(), ok, "Could not verify drive motor");
  FRC846_VERIFY(steer_.VerifyConnected(), ok, "Could not verify steer motor");
  return ok;
}

void SwerveModuleSubsystem::SetCANCoderOffset() {
  SetCANCoderOffset(GetReadings().steer_pos);
}
void SwerveModuleSubsystem::SetCANCoderOffset(units::degree_t offset) {
  SetPreferenceValue("cancoder_offset_", offset);
}

void SwerveModuleSubsystem::ZeroWithCANcoder() {
  constexpr int kMaxAttempts = 5;
  constexpr int kSleepTimeMs = 500;

  for (int attempts = 1; attempts <= kMaxAttempts; ++attempts) {
    Log("CANCoder zero attempt {}/{}", attempts, kMaxAttempts);
    auto position = cancoder_.GetAbsolutePosition();

    if (position.IsAllGood()) {
      steer_helper_.SetPosition(-position.GetValue());
      Log("Zeroed to {}!", -position.GetValue());
      return;
    }

    Warn("Attempt to zero failed, sleeping {} ms...", kSleepTimeMs);

    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
  }
  Error("Unable to zero after {} attempts", kMaxAttempts);
}

SwerveModuleReadings SwerveModuleSubsystem::ReadFromHardware() {
  SwerveModuleReadings readings;
  readings.vel = drive_helper_.GetVelocity();
  readings.drive_pos = drive_helper_.GetPosition();
  readings.steer_pos = steer_helper_.GetPosition();

  Graph("readings/drive_motor_vel", readings.vel);
  Graph("readings/drive_motor_pos", readings.drive_pos);
  Graph("readings/steer_motor_pos", readings.steer_pos);

  Graph("readings/cancoder_pos",
      units::degree_t(cancoder_.GetAbsolutePosition().GetValue()));

  return readings;
}

void SwerveModuleSubsystem::WriteToHardware(SwerveModuleTarget target) {
  if (SwerveModuleOLControlTarget* ol_target =
          std::get_if<SwerveModuleOLControlTarget>(&target)) {
    Graph("target/ol_drive_target", ol_target->drive);
    Graph("target/ol_steer_target", ol_target->steer);

    auto [steer_dir, invert_drive] =
        calculateSteerPosition(GetReadings().steer_pos, ol_target->steer);

    Graph("target/steer_dir", steer_dir);
    Graph("target/invert_drive", invert_drive);

    units::dimensionless::scalar_t cosine_comp =
        units::math::cos(steer_dir - GetReadings().steer_pos);

    Graph("target/cosine_comp", cosine_comp.to<double>());

    drive_helper_.WriteDC(
        cosine_comp *
        ol_target->drive);  // Ignore invert_drive, cosine comp should handle it

    if (std::abs(ol_target->drive) > 0.002) {
      steer_helper_.WritePosition(ol_target->steer);
    }
  } else if (SwerveModuleTorqueControlTarget* torque_target =
                 std::get_if<SwerveModuleTorqueControlTarget>(&target)) {
    // TODO: finish torque control for drivetrain
    Graph("target/torque_drive_target", torque_target->drive);
    Graph("target/torque_steer_target", torque_target->steer);
  } else {
    throw std::runtime_error("SwerveModuleTarget was not of a valid type");
  }
}

std::pair<units::degree_t, bool> SwerveModuleSubsystem::calculateSteerPosition(
    units::degree_t target_norm, units::degree_t current) {
  bool reverse = false;

  units::degree_t diff =
      frc846::math::CoterminalDifference(target_norm, current);

  if (diff > 90_deg) {
    diff -= 180_deg;
    reverse = true;
  } else if (diff < -90_deg) {
    diff += 180_deg;
    reverse = true;
  }

  return {current + diff, reverse};
}

void SwerveModuleSubsystem::SetSteerGains(
    frc846::control::base::MotorGains gains) {
  steer_.SetGains(gains);
}

}  // namespace frc846::robot::swerve