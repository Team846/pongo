#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "frc846/base/Loggable.h"
#include "frc846/robot/swerve/drivetrain.h"

namespace frc846::robot::swerve {

class AimCommand : public frc2::CommandHelper<frc2::Command, AimCommand>,
                   public frc846::base::Loggable {
public:
  AimCommand(frc846::robot::swerve::DrivetrainSubsystem* drivetrain,
      units::degree_t target);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  frc846::robot::swerve::DrivetrainSubsystem* drivetrain_;

  units::degree_t target_;
};

}  // namespace frc846::robot::swerve