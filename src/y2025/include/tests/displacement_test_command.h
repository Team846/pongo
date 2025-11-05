#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc/Timer.h>

#include "frc846/base/Loggable.h"
#include "frc846/robot/swerve/drivetrain.h"

class DisplacementTestCommand: 
        public frc2::CommandHelper<frc2::Command, DisplacementTestCommand>,
        public frc846::base::Loggable {
    
    public:
      DisplacementTestCommand(frc846::robot::swerve::DrivetrainSubsystem* drivetrain);

      void Initialize() override;
      void Execute() override;
      void End(bool interrupted) override;
      bool IsFinished() override;

    private:
        frc846::robot::swerve::DrivetrainSubsystem* drivetrain_;
        frc::Timer timer_;
        frc846::math::Vector2D start_pos_;

        units::inch_t target_distance_;
        units::unit_t<units::feet_per_second> drive_speed_;

    
                                
};