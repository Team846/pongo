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

  void OnPeriodic() override;

  void InitTeleop() override;
  void InitTest() override;

private:
  RobotContainer container_;
};
