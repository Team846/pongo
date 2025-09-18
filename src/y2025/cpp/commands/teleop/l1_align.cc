#include "commands/teleop/l1_align.h"

#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/general/l1_pos.h"
#include "commands/teleop/drive_to_reef_command.h"
#include "commands/teleop/lock_to_reef_command.h"

L1AutoAlignCommand::L1AutoAlignCommand(RobotContainer& container, bool is_left,
    units::feet_per_second_t max_speed,
    units::feet_per_second_t lower_max_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_deceleration,
    frc846::math::Vector2D& base_adj)
    : GenericCommandGroup<RobotContainer, L1AutoAlignCommand,
          frc2::SequentialCommandGroup>{container, "l1_auto_align",
          frc2::SequentialCommandGroup{
              DriveToReefCommand{container, is_left, true, max_speed,
                  max_acceleration, max_deceleration},
              /*DriveToReefCommand{&(container.drivetrain_), is_left, false,
                  max_speed, max_acceleration, max_deceleration},*/
              frc2::ParallelDeadlineGroup{
                  frc2::WaitCommand{2.5_s},
                  LockToReefCommand{container, is_left, base_adj},
                  L1PosCommand{container, false},
              },
              L1PosCommand{container, true}}} {}
