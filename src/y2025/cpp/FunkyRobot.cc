#include "FunkyRobot.h"

#include <cameraserver/CameraServer.h>
#include <frc/DSControlWord.h>
#include <frc/Filesystem.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>

#include "calculators/AntiTippingCalculator.h"
#include "commands/teleop/drive_command.h"
#include "commands/teleop/leds_command.h"
#include "control_triggers.h"
#include "field.h"
#include "frc846/wpilib/NTAction.h"
#include "rsighandler.h"

FunkyRobot::FunkyRobot() : GenericRobot{&container_} {}

void FunkyRobot::OnInitialize() {
  container_.leds_.SetDefaultCommand(LEDsCommand{container_});

  Field::Setup();

  // for (auto x : Field::getAllAutoData()) {
  //   Log("Adding Auto: {}", x.name + "_red");
  //   AddAuto(x.name + "_red", new GenericAuto{container_, x, false});
  //   Log("Adding Auto: {}", x.name + "_blue");
  //   AddAuto(x.name + "_blue", new GenericAuto{container_, x, true});
  // }

  // // Add dashboard buttons
  frc::SmartDashboard::PutData("set_cancoder_offsets",
      new frc846::wpilib::NTAction(
          [this] { container_.drivetrain_.SetCANCoderOffsets(); }));
  frc::SmartDashboard::PutData(
      "zero_bearing", new frc846::wpilib::NTAction(
                          [this] { container_.drivetrain_.ZeroBearing(); }));

  // frc::SmartDashboard::PutData(
  //     "zero_odometry", new frc846::ntinf::NTAction(
  //                          [this] { container_.drivetrain_.ZeroOdometry();
  //                          }));
}

void FunkyRobot::OnDisable() {
  container_.leds_.SetDefaultCommand(LEDsCommand{container_});
}

void FunkyRobot::InitTeleop() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});
  container_.leds_.SetDefaultCommand(LEDsCommand{container_});

  ControlTriggerInitializer::InitTeleopTriggers(container_);
}

void FunkyRobot::OnPeriodic() {
  // TODO: plug real heights into AntiTippingCalculator
  AntiTippingCalculator::SetTelescopeHeight(36_in);
  AntiTippingCalculator::SetElevatorHeight(45_in);

  auto cg = AntiTippingCalculator::CalculateRobotCG();
  Graph("robot_cg_x", cg[0]);
  Graph("robot_cg_y", cg[1]);
  Graph("robot_cg_z", cg[2]);
}

void FunkyRobot::InitTest() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  if (frc::RobotBase::IsSimulation()) configureSignalHandlers();
  return frc::StartRobot<FunkyRobot>();
}
#endif