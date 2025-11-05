#pragma once

#include "tests/displacement_test_command.h"

DisplacementTestCommand::DisplacementTestCommand(
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::base::Loggable("DisplacementTestCommand"),
      drivetrain_(drivetrain) {
    AddRequirements({drivetrain_});

    target_distance_ = 12_in; //TODO how to make this a pref instead of hard coded?

}


void DisplacementTestCommand::Initialize() {
    timer_.Reset();
    timer_.Start();

    auto pose = drivetrain_->GetReadings().pose;
    start_pos_ = pose.position;

}

void DisplacementTestCommand::Execute() {
    frc846::robot::swerve::DrivetrainOLControlTarget target;
    target.velocity = {2_fps, 0_fps}; // TODO how to make this a pref instead of hardcoded?
    target.angular_velocity = 0_deg_per_s;

    drivetrain_->SetTarget(target);
}

bool DisplacementTestCommand::IsFinished() {
    auto pose = drivetrain_->GetReadings().pose;
    frc846::math::Vector2D current_pos{
        (pose.position[0] - start_pos_[0]), 
        (pose.position[1] - start_pos_[1])};

    auto change = current_pos - start_pos_;
    units::inch_t distance_traveled = change.magnitude();

    Log("distance:", distance_traveled.value());
    Log("time:", timer_.Get().value());

    return distance_traveled >= target_distance_;

}

void DisplacementTestCommand::End(bool interrupted) {
    drivetrain_-> SetTarget(drivetrain_->ZeroTarget());
    timer_.Stop();

    Log("total time", timer_.Get().value());
}

