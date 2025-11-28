#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>

#include <memory>

#include "frc846/control/HigherMotorController.h"
#include "frc846/robot/GenericRobot.h"
#include "frc846/robot/GenericSubsystem.h"
#include "pdcsu.h"

struct ICTestReadings {
  units::degree_t pos;
  units::degrees_per_second_t vel;
};

struct ICTestTarget {
  units::degree_t pos;
};

class ICTestSubsystem
    : public frc846::robot::GenericSubsystem<ICTestReadings, ICTestTarget> {
public:
  ICTestSubsystem();
  ~ICTestSubsystem();

  void Setup() override;

  ICTestTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  ICTestReadings ReadFromHardware() override;

  void WriteToHardware(ICTestTarget target) override;

  frc846::control::config::MotorConstructionParameters motor_configs_;
  frc846::control::HigherMotorController esc_;
  std::unique_ptr<pdcsu::control::ICNORPositionControl> icnor_controller_;
  std::unique_ptr<pdcsu::util::DefArmSys> angular_sys_;
  std::shared_ptr<pdcsu::control::ICNORLearner> icnor_learner_;
};
