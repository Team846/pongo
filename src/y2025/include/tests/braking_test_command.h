// #pragma once

// #include <frc2/command/CommandHelper.h>
// #include <frc2/command/Command.h>
// #include <frc/Timer.h>

// #include "frc846/base/Loggable.h"
// #include "frc846/robot/swerve/drivetrain.h"
// #include "frc846/math/vectors.h"

// class BrakingTestCommand :
//     public frc2::CommandHelper<frc2::Command, BrakingTestCommand>, 
//     public frc846::base::Loggable {
    
//     public:
//         BrakingTestCommand(frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
//         : frc846::base::Loggable("BrakingTestCommand"),
//           drivetrain_(drivetrain) {}

//         void Initialize() override;
//         void Execute() override;
//         void End(bool interrupted) override;
//         bool IsFinished() override;

//     private:
//         frc846::robot::swerve::DrivetrainSubsystem* drivetrain_;
//         frc::Timer timer_;
//         frc846::math::Vector2D start_pos_;

//         //i change these:
//         units::feet_per_second_t initial_speed_;
//         units::second_t drive_time_; //(how long to drive before commanding stop)
//         units::inch_t max_allowed_braking_distance_; //(how far we are allowed to slide, like 8in maybe)



//     };