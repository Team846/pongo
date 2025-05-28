#include "commands/teleop/net_auto_align.h"

#include "commands/general/algal_position_command.h"
#include "commands/teleop/drive_to_net_command.h"
#include "commands/teleop/lock_to_net_command.h"

NetAutoAlignCommand::NetAutoAlignCommand(RobotContainer& container,
    units::feet_per_second_t max_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_deceleration,
    frc846::math::Vector2D& base_adj)
    : GenericCommandGroup<RobotContainer, NetAutoAlignCommand,
          frc2::SequentialCommandGroup>{container, "net_auto_align",
          frc2::SequentialCommandGroup{
              DriveToNetCommand{container, false, max_speed, max_acceleration,
                  max_deceleration},
              frc2::ParallelCommandGroup{
                  LockToNetCommand{container, base_adj},
                  AlgalPositionCommand{container, kAlgae_Net, false},
              },
              AlgalPositionCommand{container, kAlgae_Net, true},
              DriveToNetCommand{container, true, max_speed, max_acceleration,
                  max_deceleration},
              AlgalPositionCommand{container, kAlgae_Stow, false}}} {}