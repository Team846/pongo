#include "commands/teleop/complete_gpd_command.h"

#include "commands/teleop/gpd_ss_command.h"
#include "commands/teleop/lock_gpd_command.h"

CompleteGPDCommand::CompleteGPDCommand(RobotContainer& container)
    : GenericCommandGroup<RobotContainer, CompleteGPDCommand,
          frc2::ParallelDeadlineGroup>{container, "complete_gpd_command",
          frc2::ParallelDeadlineGroup{
              GPDSSCommand{container_} /*, LockGPDCommand{container_}*/
          }} {}
