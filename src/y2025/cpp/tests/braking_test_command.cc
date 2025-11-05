#include "tests/braking_test_command.h"

BrakingTestCommand::BrakingTestCommand(
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::base::Loggable("BrakingTestCommand"),
      drivetrain_(drivetrain) {
    AddRequirements({drivetrain_});

    initial_speed_ = 2_fps;
    drive_time_ = 1_s;
    max_allowed_braking_distance_ = 8_in;
}

void BrakingTestCommand::Initialize() {
    timer_.Reset();
    timer_.Start();

    start_pos_ = drivetrain_->GetReadings().pose.position;
}

void BrakingTestCommand::Execute() {
    frc846::robot::swerve::DrivetrainOLControlTarget target;

    // If still driving, set velocity to initial_speed_
    if (timer_.Get() < drive_time_) {
        target.velocity = {
            units::unit_t<units::feet_per_second>(initial_speed_),
            units::unit_t<units::feet_per_second>(0_fps)
        };
    } else {
        // Stop driving after drive_time_
        target.velocity = {0_fps, 0_fps};
    }

    target.angular_velocity = 0_deg_per_s;
    drivetrain_->SetTarget(target);
}

bool BrakingTestCommand::IsFinished() {
    auto pose = drivetrain_->GetReadings().pose;
    frc846::math::Vector2D current_pos{
        (pose.position[0] - start_pos_[0]),
        (pose.position[1] - start_pos_[1])
    };

    units::inch_t distance_traveled = current_pos.magnitude();

    Log("distance traveled", distance_traveled.value());
    Log("time", timer_.Get().value());

    // finish when distance is more than max distance
    return distance_traveled >= max_allowed_braking_distance_;
}

void BrakingTestCommand::End(bool interrupted) {
    drivetrain_->SetTarget(drivetrain_->ZeroTarget());
    timer_.Stop();

    Log("total braking time", timer_.Get().value());
}
