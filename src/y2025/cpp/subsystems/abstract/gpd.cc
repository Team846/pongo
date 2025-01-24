#include "subsystems/abstract/gpd.h"

#include <frc/DriverStation.h>
#include <units/math.h>

#include <vector>

#include "field.h"
#include "frc846/ntinf/pref.h"
#include "frc846/util/share_tables.h"

GPDSubsystem::GPDSubsystem(bool init)
    : frc846::robot::GenericSubsystem<GPDReadings, GPDTarget>("gpd", init) {
  if (init) {
#ifndef _WIN32
    for (int i = 0; i < 20; i++) {
      g_field.GetObject(std::to_string(i));
    }
    frc::SmartDashboard::PutData("NoteField", &g_field);
#endif
  }
}

GPDTarget GPDSubsystem::ZeroTarget() const {
  GPDTarget target;
  return target;
}

bool GPDSubsystem::VerifyHardware() { return true; }

frc846::math::Vector2D GPDSubsystem::findDistance(units::degree_t theta_h,
                                                  units::degree_t theta_v) {
  units::foot_t height = mount_height_.value() - algae_height_.value();
  auto dist = units::math::abs(height * units::math::tan(theta_v));
  auto yDist = dist * units::math::cos(theta_h);
  auto xDist = dist * units::math::sin(theta_h);

  return {xDist, yDist};
}

GPDReadings GPDSubsystem::ReadFromHardware() {
  GPDReadings readings{};

  units::degree_t bearing_ =
      units::degree_t(GetPreferenceValue_double("robot_bearing_"));

  units::inch_t robot_x =
      units::foot_t(GetPreferenceValue_double("odometry_x_"));

  units::inch_t robot_y =
      units::foot_t(GetPreferenceValue_double("odometry_y_"));

  units::feet_per_second_t velocity_x = units::feet_per_second_t(
      GetPreferenceValue_double("velocity_x_"));

  units::feet_per_second_t velocity_y = units::feet_per_second_t(
     GetPreferenceValue_double("velocity_y_"));

  std::vector<double> theta_hs = gpdTable->GetNumberArray("theta_h", {});
  std::vector<double> theta_vs = gpdTable->GetNumberArray("theta_v", {});
  auto latency = gpdTable->GetNumber("latency", 0.1) * 1_s;

  for (size_t i = 0; i < theta_hs.size(); i++) {
    auto distance = findDistance(units::degree_t(theta_hs[i]),
                                 units::degree_t(theta_vs[i]));
    auto pos =
        distance.rotate(bearing_) + frc846::math::Vector2D{robot_x, robot_y} +
        frc846::math::Vector2D{velocity_x * latency, velocity_y * latency};

    readings.notes.push_back(pos);
  }

  int num_notes = readings.notes.size();
  Graph("notes_detected", num_notes);

#ifndef _WIN32
  for (int i = 0; i < std::min(20, num_notes); i++) {
    auto pos = readings.notes[i];
    g_field.getPoint(std::to_string(i))
        ->SetPose(pos[0], pos[1], frc::Rotation2d(0_deg));
  }
  for (int i = std::min(20, num_notes); i < 20; i++) {
    g_field.GetObject(std::to_string(i))
        ->SetPose(5000_ft, 5000_ft, frc::Rotation2d(0_deg));
  }
#endif

  return readings;
}

void GPDSubsystem::WriteToHardware(GPDTarget target) {}