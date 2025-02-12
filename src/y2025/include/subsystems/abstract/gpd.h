#pragma once

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>

#include "frc846/base/Loggable.h"
#include "frc846/math/Smoother.h"
#include "frc846/math/vectors.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "ports.h"

struct GPDTarget {};

struct gp_past_loc {
  std::chrono::milliseconds time;
  frc846::math::Vector2D position;
};

struct gp_track {
  int id;                           // Unique identifier for the ball
  frc846::math::Vector2D position;  // Last known position
  frc846::math::VectorND<units::feet_per_second, 2> velocity;
  int missedFrames;  // Counter for missed detections
  std::vector<gp_past_loc> pastPositions;
};

struct GPDReadings {
  std::vector<frc846::math::Vector2D> gamepieces;
  std::vector<gp_track> tracks;
};

class GPDSubsystem
    : public frc846::robot::GenericSubsystem<GPDReadings, GPDTarget> {
public:
  GPDSubsystem(frc846::robot::swerve::DrivetrainSubsystem* drivetrain);

  std::pair<frc846::math::Vector2D, bool> getBestGP(
      const std::vector<frc846::math::Vector2D> algae);

  void Setup() override;

  GPDTarget ZeroTarget() const override;

  std::vector<gp_track> update(std::vector<frc846::math::Vector2D>& detections);

  frc846::math::VectorND<units::feet_per_second, 2> calculateVelocity(
      std::vector<gp_past_loc> pastPositions);

  bool VerifyHardware() override;

private:
  frc::Field2d g_field;

  const units::inch_t maxDistance = 1000000_in;
  int max_missed_frames = 50;
  std::vector<gp_track> tracks;
  int nextId = 1;

  frc846::math::Smoother smoothervx{0.01}; 
  frc846::math::Smoother smoothervy{0.01};

  std::shared_ptr<nt::NetworkTable> gpdTable =
      nt::NetworkTableInstance::GetDefault().GetTable("GPDCam1");

  GPDReadings ReadFromHardware() override;

  void WriteToHardware(GPDTarget target) override;

  frc846::robot::swerve::DrivetrainSubsystem* drivetrain_;

  units::degree_t gp_spin_;
};
