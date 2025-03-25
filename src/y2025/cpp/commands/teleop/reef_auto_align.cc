#include "commands/teleop/reef_auto_align.h"

#include <frc2/command/WaitCommand.h>

#include "commands/teleop/drive_to_reef_command.h"
#include "commands/teleop/lock_to_reef_command.h"

ReefAutoAlignCommand::ReefAutoAlignCommand(RobotContainer& container,
    bool is_left, units::feet_per_second_t max_speed,
    units::feet_per_second_t lower_max_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_deceleration,
    frc846::math::Vector2D& base_adj)
    : GenericCommandGroup<RobotContainer, ReefAutoAlignCommand,
          frc2::SequentialCommandGroup>{container, "reef_auto_align",
          frc2::SequentialCommandGroup{
              DriveToReefCommand{&(container.drivetrain_), is_left, true,
                  max_speed, max_acceleration, max_deceleration},
              /*DriveToReefCommand{&(container.drivetrain_), is_left, false,
                  max_speed, max_acceleration, max_deceleration},*/
              LockToReefCommand{container, is_left, base_adj}}} {}
