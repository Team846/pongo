#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "frc846/base/Loggable.h"
#include "frc846/math/fieldpoints.h"
#include "frc846/robot/swerve/drivetrain.h"

namespace frc846::robot::swerve {

class DriveToPointCommand
    : public frc2::CommandHelper<frc2::Command, DriveToPointCommand>,
      public frc846::base::Loggable {
public:
  DriveToPointCommand(frc846::robot::swerve::DrivetrainSubsystem* drivetrain,
      frc846::math::FieldPoint target, units::feet_per_second_t max_speed,
      units::feet_per_second_squared_t max_acceleration,
      units::feet_per_second_squared_t max_deceleration);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  frc846::robot::swerve::DrivetrainSubsystem* drivetrain_;

  units::feet_per_second_t max_speed_;
  units::feet_per_second_squared_t max_acceleration_;
  units::feet_per_second_squared_t max_deceleration_;

  frc846::math::FieldPoint target_;
  frc846::math::Vector2D start_point_;

  bool is_decelerating_ = false;
};

}  // namespace frc846::robot::swerve