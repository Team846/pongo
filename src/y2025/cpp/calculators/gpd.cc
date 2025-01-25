#include "calculators/gpd.h"

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

#include <vector>

#include "frc846/robot/swerve/odometry/swerve_pose.h"

GPDCalculator::GPDCalculator() {
#ifndef _WIN32
  for (int i = 0; i < 20; i++) {
    g_field.GetObject(std::to_string(i));
  }
  frc::SmartDashboard::PutData("NoteField", &g_field);
#endif
}

std::pair<frc846::math::Vector2D, int> GPDCalculator::getBestGP(
    const std::vector<frc846::math::Vector2D> algae,
    const frc846::math::VectorND<units::feet_per_second, 2> robot_velocity) {
  if (algae.empty()) { return {}; }
  frc846::math::Vector2D closest_algae;
  int closest_algae_index = -1;
  double max_dot_product = -std::numeric_limits<double>::infinity();
  std::pair<double, double> robot_unit_vector = std::pair<double, double>{
      robot_velocity.unit().toPair().first.to<double>(),
      robot_velocity.unit().toPair().second.to<double>()};

  for (size_t i = 0; i < algae.size();
       i++) {  // find closest matching vector in
               // terms of angle to robot velocity
    frc846::math::Vector2D relative_note = algae.at(i);
    std::pair<double, double> note_unit_vector{
        robot_velocity.unit().toPair().first.to<double>() /
            relative_note.magnitude().to<double>(),
        robot_velocity.unit().toPair().second.to<double>() /
            relative_note.magnitude().to<double>()};
    double dot_product = robot_unit_vector.first * note_unit_vector.first +
                         robot_unit_vector.second * note_unit_vector.second;
    if (dot_product > max_dot_product) {
      // std::cout << "x: " << relative_note.unit().toPair().first << std::endl;
      //    std::cout   << "y: " << relative_note.unit().toPair().second <<
      //    std::endl;
      max_dot_product = dot_product;
      closest_algae = relative_note;
      closest_algae_index = i;
    }
  }
  //   std::cout<<"x"<<closest_note.x.to<double>()<<"y"<<closest_note.y.to<double>()<<std::endl;
  return std::pair<frc846::math::Vector2D, int>{
      closest_algae, closest_algae_index};
}

std::pair<bool, frc846::math::Vector2D> GPDCalculator::calculate(
    std::vector<frc846::math::Vector2D> gp, std::vector<double> theta_x,
    frc846::robot::swerve::odometry::SwervePose pose, units::second_t latency,
    std::vector<double> distances)

{
  gp.clear();

  units::degree_t bearing_ = pose.bearing;
  units::inch_t robot_x = pose.position[0];
  units::inch_t robot_y = pose.position[1];
  units::feet_per_second_t velocity_x = pose.velocity[0];
  units::feet_per_second_t velocity_y = pose.velocity[1];

  for (size_t i = 0; i < distances.size() && i < theta_x.size(); ++i) {
    gp.push_back(
        frc846::math::Vector2D{units::foot_t(distances[i]),
            bearing_ + units::degree_t(theta_x[i]), true} +
        frc846::math::Vector2D{robot_x, robot_y} +
        frc846::math::Vector2D{velocity_x * latency, velocity_y * latency});
  }  // Add missing include

  int num_gps = gp.size();

#ifndef _WIN32
  for (int i = 0; i < std::min(20, num_gps); i++) {
    auto pos = gp[i];
    g_field.GetObject(std::to_string(i))->SetPose(pos[0], pos[1], 0_deg);
  }

#endif

  if (gp.empty()) {
    return std::pair<bool, frc846::math::Vector2D>{
        false, frc846::math::Vector2D{}};
  }

  frc846::math::Vector2D best_algae = getBestGP(gp,
      frc846::math::VectorND<units::feet_per_second, 2>{velocity_x, velocity_y})
                                          .first;

  return std::pair<bool, frc846::math::Vector2D>{true, best_algae};
}