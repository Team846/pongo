#pragma once

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <hal/Types.h>
#include <units/time.h>

#include "frc846/robot/GenericRobotContainer.h"

namespace frc846::robot {

enum Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

class GenericRobot : public frc::RobotBase, public frc846::base::Loggable {
public:
  static constexpr auto kPeriod = 10_ms;  // 100Hz

  GenericRobot(GenericRobotContainer* container);

  ~GenericRobot() override;

  void StartCompetition() override final;
  void EndCompetition() override final;

  virtual void OnInitialize() = 0;

  virtual void OnDisable() = 0;

  virtual void OnPeriodic() = 0;

  virtual void InitTeleop() = 0;
  virtual void InitTest() = 0;

  void VerifyHardware();

  void AddAuto(std::string name, frc2::Command* command);

private:
  hal::Handle<HAL_NotifierHandle> notifier_;
  units::microsecond_t next_loop_time_;

  Mode last_mode_;

private:
  GenericRobotContainer* generic_robot_container_;

  frc2::Command* auto_command_ = nullptr;
  frc::SendableChooser<std::string> auto_chooser_;
  std::unordered_map<std::string, frc2::Command*> autos_;
};

}  // namespace frc846::robot
