#pragma once

#include <frc/smartdashboard/Field2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>

#include "frc846/base/Loggable.h"
#include "frc846/math/vectors.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/robot/swerve/odometry/swerve_pose.h"
#include "frc846/wpilib/NTAction.h"
#include "ports.h"

class GPDCalculator {
public:
  GPDCalculator();

  std::pair<frc846::math::Vector2D, int> getBestGP(
      const std::vector<frc846::math::Vector2D> algae,
      const frc846::math::VectorND<units::feet_per_second, 2> robot_velocity);

  std::pair<bool, frc846::math::Vector2D> calculate(
      std::vector<frc846::math::Vector2D> gp, std::vector<double> theta_x,
      frc846::robot::swerve::odometry::SwervePose pose, units::second_t latency,
      std::vector<double> distances);

private:
  frc::Field2d g_field;

  frc846::base::Loggable algo_params{"algo_parameters"};

  std::shared_ptr<nt::NetworkTable> gpdTable =
      nt::NetworkTableInstance::GetDefault().GetTable("GPDCam1");
};