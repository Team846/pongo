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

#include "autos/auton_seqs.h"
#include "calculators/AntiTippingCalculator.h"
#include "commands/teleop/algal_command.h"
#include "commands/teleop/climber_command.h"
#include "commands/teleop/coral_command.h"
#include "commands/teleop/drive_command.h"
#include "control_triggers.h"
#include "field.h"
#include "frc846/wpilib/NTAction.h"
#include "rsighandler.h"
#include "subsystems/hardware/leds_logic.h"

FunkyRobot::FunkyRobot() : GenericRobot{&container_} {}

void FunkyRobot::OnInitialize() {
  Field::Setup();

  // for (auto x : Field::getAllAutoData()) {
  //   Log("Adding Auto: {}", x.name + "_red");
  //   AddAuto(x.name + "_red", new GenericAuto{container_, x, false});
  //   Log("Adding Auto: {}", x.name + "_blue");
  //   AddAuto(x.name + "_blue", new GenericAuto{container_, x, true});
  // }

  ADD_AUTO_VARIANTS(ThreePieceAuto, "3PC");
  ADD_AUTO_VARIANTS(OnePieceAndNetAuto, "1PCN");
  ADD_AUTO_VARIANTS(LeaveAuto, "LEAVE");

  // // Add dashboard buttons
  frc::SmartDashboard::PutData("set_cancoder_offsets",
      new frc846::wpilib::NTAction(
          [this] { container_.drivetrain_.SetCANCoderOffsets(); }));
  frc::SmartDashboard::PutData(
      "zero_bearing", new frc846::wpilib::NTAction(
                          [this] { container_.drivetrain_.ZeroBearing(); }));

  frc::SmartDashboard::PutData("zero_odometry",
      new frc846::wpilib::NTAction(
          [this] { container_.drivetrain_.SetPosition({0_in, 0_in}); }));
}

void FunkyRobot::OnDisable() {}

void FunkyRobot::InitTeleop() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});

  container_.coral_ss_.SetDefaultCommand(CoralCommand{container_});
  container_.algal_ss_.SetDefaultCommand(AlgalCommand{container_});
  container_.climber_.SetDefaultCommand(ClimberCommand{container_});

  ControlTriggerInitializer::InitTeleopTriggers(container_);
}

void FunkyRobot::OnPeriodic() {
  LEDsLogic::UpdateLEDs(&container_);

  // TODO: fix AntiTippingCalculator cg calc from heights
  AntiTippingCalculator::SetTelescopeHeight(
      container_.coral_ss_.telescope.GetReadings().position);
  AntiTippingCalculator::SetElevatorHeight(
      container_.algal_ss_.elevator.GetReadings().position);

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