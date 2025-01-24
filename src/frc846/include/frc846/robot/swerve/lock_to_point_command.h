#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "frc846/base/Loggable.h"
#include "frc846/math/fieldpoints.h"
#include "frc846/robot/swerve/drivetrain.h"

namespace frc846::robot::swerve {

class LockToPointCommand
    : public frc2::CommandHelper<frc2::Command, LockToPointCommand>,
      public frc846::base::Loggable {
public:
  LockToPointCommand(frc846::robot::swerve::DrivetrainSubsystem* drivetrain,
      frc846::math::FieldPoint target);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  frc846::robot::swerve::DrivetrainSubsystem* drivetrain_;

  frc846::math::FieldPoint target_;
};

}  // namespace frc846::robot::swerve