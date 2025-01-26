#include "subsystems/abstract/gpd.h"

#include <vector>

#include "frc846/math/fieldpoints.h"

GPDSubsystem::GPDSubsystem(
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::robot::GenericSubsystem<GPDReadings, GPDTarget>{"GPD"},
      drivetrain_{drivetrain} {
#ifndef _WIN32
  for (int i = 0; i < 20; i++) {
    g_field.GetObject(std::to_string(i));
  }
  frc::SmartDashboard::PutData("NoteField", &g_field);
#endif
}

GPDTarget GPDSubsystem::ZeroTarget() const { return GPDTarget{}; }

bool GPDSubsystem::VerifyHardware() { return true; }

void GPDSubsystem::Setup() {}

GPDReadings GPDSubsystem::ReadFromHardware() {
  GPDReadings readings;
  frc846::robot::swerve::DrivetrainReadings drivetrain_readings =
      drivetrain_->GetReadings();

  g_field.SetRobotPose(frc846::math::FieldPoint::field_size_y -
                           drivetrain_readings.pose.position[1],
      drivetrain_readings.pose.position[0],
      180_deg - drivetrain_readings.pose.bearing);

  std::vector<double> distances = gpdTable->GetNumberArray("distances", {});
  units::second_t latency = gpdTable->GetNumber("tl", 0.05) *
                            1_s;  // TODO: check when entry was last updated
  std::vector<double> theta_x = gpdTable->GetNumberArray("tx", {});

  readings.gamepieces.clear();

  for (size_t i = 0; i < distances.size() && i < theta_x.size(); ++i) {
    readings.gamepieces.push_back(
        frc846::math::Vector2D{units::inch_t(distances[i]),
            drivetrain_readings.pose.bearing + units::degree_t(theta_x[i]),
            true} +
        drivetrain_readings.pose.position +
        frc846::math::Vector2D{drivetrain_readings.pose.velocity[0] * latency,
            drivetrain_readings.pose.velocity[1] * latency});
  }

  int num_gps = readings.gamepieces.size();

#ifndef _WIN32
  for (int i = 0; i < std::min(20, num_gps); i++) {
    auto pos = readings.gamepieces[i];
    g_field.GetObject(std::to_string(i))
        ->SetPose(
            frc846::math::FieldPoint::field_size_y - pos[1], pos[0], 0_deg);
  }
  for (int i = std::min(20, num_gps); i < 20; i++) {
    g_field.GetObject(std::to_string(i))->SetPose(100_m, 100_m, 0_deg);
  }
#endif
  return readings;
}

void GPDSubsystem::WriteToHardware(GPDTarget target) {}