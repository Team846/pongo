#include "subsystems/abstract/gpd.h"

#include <units/math.h>

#include <vector>

GPDSubsystem::GPDSubsystem()
    : frc846::robot::GenericSubsystem<GPDReadings, GPDTarget>("gpd") {
#ifndef _WIN32
  for (int i = 0; i < 20; i++) {
    g_field.GetObject(std::to_string(i));
  }
  frc::SmartDashboard::PutData("NoteField", &g_field);
#endif
}

GPDTarget GPDSubsystem::ZeroTarget() const {
  GPDTarget target;
  return target;
}

bool GPDSubsystem::VerifyHardware() { return true; }

std::pair<frc846::math::Vector2D, int> getBestGP(
    std::vector<frc846::math::Vector2D>& notes,
    frc846::math::Vector2D& robot_velocity) {
  return {frc846::math::Vector2D{0.0_in, 0.0_deg, true}, 0};
}

GPDReadings GPDSubsystem::ReadFromHardware() {
  GPDReadings readings{};

  units::degree_t bearing_ =
      GetPreferenceValue_unit_type<units::degree_t>("bearing");
  units::inch_t robot_x =
      GetPreferenceValue_unit_type<units::inch_t>("position_x");
  units::inch_t robot_y =
      GetPreferenceValue_unit_type<units::inch_t>("position_y");
  units::feet_per_second_t velocity_x =
      GetPreferenceValue_unit_type<units::feet_per_second_t>("velocity_x");
  units::feet_per_second_t velocity_y =
      GetPreferenceValue_unit_type<units::feet_per_second_t>("velocity_y");

  std::vector<double> theta_x = gpdTable->GetNumberArray("tx", {});
  std::vector<double> distances = gpdTable->GetNumberArray("distances", {});
  units::second_t latency = gpdTable->GetNumber("tl", 0.1) * 1_s;

  readings.gp.clear();

  for (size_t i = 0; i < distances.size() && i < theta_x.size(); ++i) {
    readings.gp.push_back(
        frc846::math::Vector2D{units::foot_t(distances[i]),
            bearing_ + units::degree_t(theta_x[i]), true} +
        frc846::math::Vector2D{robot_x, robot_y} +
        frc846::math::Vector2D{velocity_x * latency, velocity_y * latency});
  }

  int num_gps = readings.gp.size();
  Graph("notes_detected", num_gps);

  if (num_gps > 0) { readings.gp_detected = true; }

#ifndef _WIN32
  for (int i = 0; i < std::min(20, num_gps); i++) {
    auto pos = readings.gp[i];
    g_field.GetObject(std::to_string(i))->SetPose(pos[0], pos[1], 0_deg);
  }
  // for (int i = std::min(20, num_gps); i < 20; i++) {
  //   g_field.GetObject(std::to_string(i))
  //       ->SetPose(5000_ft, 5000_ft, 0_deg);
  // }
#endif

  return readings;
}

void GPDSubsystem::WriteToHardware(GPDTarget target) {}