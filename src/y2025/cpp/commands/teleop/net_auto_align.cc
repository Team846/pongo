#include "commands/teleop/net_auto_align.h"

#include "commands/general/algal_position_command.h"
#include "commands/teleop/lock_to_net_command.h"

NetAutoAlignCommand::NetAutoAlignCommand(
    RobotContainer& container, frc846::math::Vector2D& base_adj)
    : GenericCommandGroup<RobotContainer, NetAutoAlignCommand,
          frc2::SequentialCommandGroup>{container, "net_auto_align",
          frc2::SequentialCommandGroup{
              LockToNetCommand{container, base_adj, false},
              AlgalPositionCommand{container, kAlgae_Net, true},
              LockToNetCommand{container, base_adj, true},
              AlgalPositionCommand{container, kAlgae_Stow, true}}} {}
