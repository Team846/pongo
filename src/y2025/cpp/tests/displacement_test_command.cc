#include "tests/displacement_test_command.h"
#include "frc846/robot/swerve/drivetrain.h"

DisplacementTestCommand::DisplacementTestCommand(
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::base::Loggable("DisplacementTestCommand"),
      drivetrain_(drivetrain)    
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

    double duty_cycle = drivetrain_->GetPreferenceValue_double("displacement_test/duty_cycle");
    drivetrain_->WriteToHardware(duty_cycle);

    auto pose = drivetrain_->GetReadings().pose;

    frc846::math::Vector2D delta{
        (pose.position[0] - start_pos_[0]),
        (pose.position[1] - start_pos_[1])
    };
    units::inch_t distance_traveled = delta.magnitude();

    if (distance_traveled >= drivetrain_->GetPreferenceValue_unit_type<units::inch_t>("displacement_test/target_distance"))
    {
        Log("target distance time", timer_.Get().value());

        drivetrain_->WriteToHardware(-1 * duty_cycle);

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

    return (drivetrain_->GetReadings().pose.velocity.magnitude() < GetPreferenceValue_unit_type<units::feet_per_second_t>("displacement_test/close_to_zero_velocity")) 
            && (0_fps < drivetrain_->GetReadings().pose.velocity.magnitude());
}

void DisplacementTestCommand::End(bool interrupted) {
    
    drivetrain_->WriteToHardware(0.0);
    timer_.Stop();

    Log("total time", timer_.Get().value());
}
