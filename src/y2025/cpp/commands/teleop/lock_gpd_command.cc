#include "commands/teleop/lock_gpd_command.h"

#include "reef.h"

LockGPDCommand::LockGPDCommand(RobotContainer& container)
    : LockToPointCommand{&(container.drivetrain_), {}}, container_{container} {}

std::pair<frc846::math::FieldPoint, bool> LockGPDCommand::GetTargetPoint() {
  auto gpd_readings = container_.GPD_.GetReadings();
  const auto [gpd_pos, valid] =
      container_.GPD_.getBestGP(gpd_readings.gamepieces);

  if (!valid) return {{}, false};

  return {{gpd_pos,
              (gpd_pos - drivetrain_->GetReadings().pose.position).angle(true),
              0_fps},
      true};
}