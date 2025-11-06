#include "tests/displacement_test_command.h"
#include "frc846/robot/swerve/drivetrain.h"

DisplacementTestCommand::DisplacementTestCommand(
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::base::Loggable("DisplacementTestCommand"),
      drivetrain_(drivetrain),
      target_distance_(12_in)        // default 12 inches
    //   drive_speed_(0.2_fps)       
{
    AddRequirements({drivetrain_});
}

void DisplacementTestCommand::Initialize() {
    timer_.Reset();
    timer_.Start();

    auto pose = drivetrain_->GetReadings().pose;
    start_pos_ = pose.position;  
}

void DisplacementTestCommand::Execute() {

    drivetrain_->WriteToHardware(0.2);

    auto pose = drivetrain_->GetReadings().pose;

    frc846::math::Vector2D delta{
        (pose.position[0] - start_pos_[0]),
        (pose.position[1] - start_pos_[1])
    };
    units::inch_t distance_traveled = delta.magnitude();

    if (distance_traveled >= target_distance_)
    {
        Log("target distance time", timer_.Get().value());

        drivetrain_->WriteToHardware(-0.2);

    }
}

bool DisplacementTestCommand::IsFinished() {
    auto pose = drivetrain_->GetReadings().pose;

    frc846::math::Vector2D delta{
        (pose.position[0] - start_pos_[0]),
        (pose.position[1] - start_pos_[1])
    };

    units::inch_t distance_traveled = delta.magnitude();

    Log("distance", distance_traveled.value());
    Log("time", timer_.Get().value());


    return (drivetrain_->GetReadings().pose.velocity.magnitude() < 0.1_fps) && (0_fps < drivetrain_->GetReadings().pose.velocity.magnitude());
}

void DisplacementTestCommand::End(bool interrupted) {
    
    drivetrain_->WriteToHardware(0.0);
    timer_.Stop();

    Log("total time", timer_.Get().value());
}
