#include "tests/displacement_test_command.h"

DisplacementTestCommand::DisplacementTestCommand(
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::base::Loggable("DisplacementTestCommand"),
      drivetrain_(drivetrain),
      target_distance_(12_in),        // default 12 inches
      drive_speed_(0.2_fps)       
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
    frc846::math::VectorND<units::feet_per_second, 2> velocity_vec{0_fps, drive_speed_};

    frc846::robot::swerve::DrivetrainOLControlTarget target;
    target.velocity = velocity_vec;
    target.angular_velocity = 0_deg_per_s;

    drivetrain_->SetTarget(target);
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

    return distance_traveled >= target_distance_;
}

void DisplacementTestCommand::End(bool interrupted) {
    drivetrain_->SetTarget(drivetrain_->ZeroTarget());
    timer_.Stop();

    Log("total time", timer_.Get().value());
}
