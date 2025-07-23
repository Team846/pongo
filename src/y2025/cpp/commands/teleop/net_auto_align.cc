#include "commands/teleop/net_auto_align.h"

#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include "commands/general/algal_position_command.h"
#include "commands/teleop/drive_to_net_command.h"

NetAutoAlignCommand::NetAutoAlignCommand(RobotContainer& container,
    units::feet_per_second_t max_speed, units::feet_per_second_t max_net_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_net_acceleration,
    units::feet_per_second_squared_t max_deceleration,
    units::feet_per_second_squared_t max_net_deceleration)
    : GenericCommandGroup<RobotContainer, NetAutoAlignCommand,
          frc2::SequentialCommandGroup>{container, "net_auto_align",
          frc2::SequentialCommandGroup{
              DriveToNetCommand{container, false, max_speed, max_acceleration,
                  max_deceleration},
              AlgalPositionCommand{container, kAlgae_Net, false},
              DriveToNetCommand{container, true, max_net_speed,
                  max_net_acceleration, max_net_deceleration},
              frc2::ParallelCommandGroup{
                  AlgalPositionCommand{container, kAlgae_Net, true},
                  frc2::WaitUntilCommand{[&] {
                    return !container_.algal_ss_.algal_end_effector
                                .GetReadings()
                                .has_piece_;
                  }},
              },
              frc2::WaitCommand{1.0_s},
              DriveToNetCommand{container, false, max_net_speed,
                  max_net_acceleration, max_net_deceleration},
              frc2::WaitCommand{0.5_s},
              DriveToNetCommand{container, false, max_net_speed,
                  max_net_acceleration, max_net_deceleration},
              AlgalPositionCommand{container, kAlgae_Stow, false}}} {}