#pragma once

#include <frc/DigitalInput.h>

#include "autos/GenericAuto.h"
#include "frc846/robot/GenericRobot.h"
#include "subsystems/robot_container.h"

class FunkyRobot : public frc846::robot::GenericRobot {
public:
  FunkyRobot();

  void OnInitialize() override;

  void OnDisable() override;
  void OnEnable() override;

  void OnPeriodic() override;

  void InitTeleop() override;
  void InitTest() override;

private:
  RobotContainer container_;

  frc::DigitalInput home_switch_{0};
  frc::DigitalInput coast_switch_{1};
  frc::DigitalInput gyro_switch_{2};

  int coast_count_{0};
  int homing_count_gyro{0};
  int homing_count_{0};
};
