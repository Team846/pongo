#pragma once

#include <networktables/NetworkTable.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/math.h>
#include <units/velocity.h>

#include <map>
#include <memory>

#include "frc846/math/calculator.h"
#include "frc846/math/vectors.h"
#include "frc846/robot/swerve/odometry/swerve_pose.h"

namespace frc846::robot::calculators {

struct ATCalculatorInput {
  frc846::robot::swerve::odometry::SwervePose pose;
  frc846::robot::swerve::odometry::SwervePose old_pose;
  units::degrees_per_second_t angular_velocity;

  double aprilVarianceCoeff;
  units::second_t fudge_latency;
  units::second_t bearing_latency;
};

struct ATCalculatorOutput {
  frc846::math::VectorND<units::inch, 2> pos;
  double variance;
};

struct AprilTagData {
  units::inch_t x_pos;
  units::inch_t y_pos;
};

struct ATCalculatorConstants {
  std::map<int, AprilTagData> tag_locations;

  std::vector<units::inch_t> camera_x_offsets;
  std::vector<units::inch_t> camera_y_offsets;

  int cams;

  std::vector<std::shared_ptr<nt::NetworkTable>> april_tables;
};

class AprilTagCalculator : public frc846::math::Calculator<ATCalculatorInput,
                               ATCalculatorOutput, ATCalculatorConstants> {
public:
  AprilTagCalculator() {};

  ATCalculatorOutput calculate(ATCalculatorInput input) override;

private:
  frc846::math::VectorND<units::inch, 2> correction;
  frc846::math::VectorND<units::inch, 2> getPos(units::degree_t bearing,
      units::degree_t theta, units::inch_t distance, int tag, int camera);
};
}