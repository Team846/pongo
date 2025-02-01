#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "frc846/base/Loggable.h"
#include "frc846/math/fieldpoints.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "subsystems/robot_container.h"

namespace frc846::robot::swerve {

class LockToPointCommand
    : public frc2::CommandHelper<frc2::Command, LockToPointCommand>,
      public frc846::base::Loggable {
public:
  LockToPointCommand(DrivetrainSubsystem* drivetrain,
      frc846::math::FieldPoint target,
      std::function<std::pair<frc846::math::FieldPoint, bool>(
          frc846::math::FieldPoint ctarget, frc846::math::FieldPoint start,
          bool firstLoop)>
          updateTarget = nullptr);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

protected:
  frc846::robot::swerve::DrivetrainSubsystem* drivetrain_;

  frc846::math::FieldPoint target_{};
  frc846::math::FieldPoint start_{};

  bool first_loop_ = true;

private:
  std::function<std::pair<frc846::math::FieldPoint, bool>(
      frc846::math::FieldPoint ctarget, frc846::math::FieldPoint start,
      bool firstLoop)>
      updateTarget_;
};

}  // namespace frc846::robot::swerve