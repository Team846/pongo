#include "commands/teleop/processor_auto_align.h"

#include "commands/teleop/drive_to_processor_command.h"
#include "commands/teleop/lock_to_processor_command.h"

ProcessorAutoAlignCommand::ProcessorAutoAlignCommand(RobotContainer& container,
    units::feet_per_second_t max_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_deceleration,
    frc846::math::Vector2D& base_adj)
    : GenericCommandGroup<RobotContainer, ProcessorAutoAlignCommand,
          frc2::SequentialCommandGroup>{container, "Processor_auto_align",
          frc2::SequentialCommandGroup{
              DriveToProcessorCommand{&(container.drivetrain_), max_speed,
                  max_acceleration, max_deceleration},
              LockToProcessorCommand{container, base_adj}}} {}
