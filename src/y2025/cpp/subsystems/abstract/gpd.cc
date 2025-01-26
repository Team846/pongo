#include "subsystems/abstract/gpd.h"

#include <vector>

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

// std::pair<frc846::math::Vector2D, int> GPDSubsystem::getBestGP(
//     const std::vector<frc846::math::Vector2D> algae,
//     const frc846::math::VectorND<units::feet_per_second, 2> robot_velocity) {
//   if (algae.empty()) { return {}; }
//   frc846::math::Vector2D closest_algae;
//   int closest_algae_index = -1;
//   double max_dot_product = -std::numeric_limits<double>::infinity();
//   std::pair<double, double> robot_unit_vector = std::pair<double, double>{
//       robot_velocity.unit().toPair().first.to<double>(),
//       robot_velocity.unit().toPair().second.to<double>()};

//   for (size_t i = 0; i < algae.size();
//        i++) {  // find closest matching vector in
//                // terms of angle to robot velocity
//     frc846::math::Vector2D relative_note = algae.at(i);
//     std::pair<double, double> note_unit_vector{
//         robot_velocity.unit().toPair().first.to<double>() /
//             relative_note.magnitude().to<double>(),
//         robot_velocity.unit().toPair().second.to<double>() /
//             relative_note.magnitude().to<double>()};
//     double dot_product = robot_unit_vector.first * note_unit_vector.first +
//                          robot_unit_vector.second * note_unit_vector.second;
//     if (dot_product > max_dot_product) {
//       max_dot_product = dot_product;
//       closest_algae = relative_note;
//       closest_algae_index = i;
//     }
//   }
//   return std::pair<frc846::math::Vector2D, int>{
//       closest_algae, closest_algae_index};
// }

GPDTarget GPDSubsystem::ZeroTarget() const { return GPDTarget{}; }

bool GPDSubsystem::VerifyHardware() { return true; }

void GPDSubsystem::Setup() {}

GPDReadings GPDSubsystem::ReadFromHardware() {
  GPDReadings readings;
  frc846::robot::swerve::DrivetrainReadings drivetrain_readings =
      drivetrain_->GetReadings();

  std::vector<double> distances = gpdTable->GetNumberArray("distances", {});
  units::second_t latency = gpdTable->GetNumber("tl", 0.1) * 1_s;
  std::vector<double> theta_x = gpdTable->GetNumberArray("tx", {});

  readings.gamepieces.clear();

  for (size_t i = 0; i < distances.size() && i < theta_x.size(); ++i) {
    readings.gamepieces.push_back(
        frc846::math::Vector2D{units::foot_t(distances[i]),
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
    g_field.GetObject(std::to_string(i))->SetPose(pos[0], pos[1], 0_deg);
  }
#endif
}

void GPDSubsystem::WriteToHardware(GPDTarget target) {}