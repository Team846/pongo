#pragma once

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>

#include "frc846/base/Loggable.h"
#include "frc846/math/vectors.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "ports.h"

struct GPDTarget {};

struct GPDReadings {
  std::vector<frc846::math::Vector2D> gamepieces;
};

class GPDSubsystem
    : public frc846::robot::GenericSubsystem<GPDReadings, GPDTarget> {
public:
  GPDSubsystem(frc846::robot::swerve::DrivetrainSubsystem* drivetrain);

  std::pair<frc846::math::Vector2D, bool> getBestGP(
      const std::vector<frc846::math::Vector2D> algae);

  void Setup() override;

  GPDTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  frc::Field2d g_field;

  std::shared_ptr<nt::NetworkTable> gpdTable =
      nt::NetworkTableInstance::GetDefault().GetTable("GPDCam1");

  GPDReadings ReadFromHardware() override;

  void WriteToHardware(GPDTarget target) override;

  frc846::robot::swerve::DrivetrainSubsystem* drivetrain_;

  units::degree_t gp_spin_;
};