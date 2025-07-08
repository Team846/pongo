#include "FunkyRobot.h"

#include <cameraserver/CameraServer.h>
#include <frc/DSControlWord.h>
#include <frc/Filesystem.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>

#include "autos/auton_seqs.h"
#include "calculators/AntiTippingCalculator.h"
#include "commands/general/algal_position_command.h"
#include "commands/general/coral_position_command.h"
#include "commands/teleop/algal_command.h"
#include "commands/teleop/climber_command.h"
#include "commands/teleop/coral_command.h"
#include "commands/teleop/drive_command.h"
#include "control_triggers.h"
#include "field.h"
#include "frc846/wpilib/NTAction.h"
#include "rsighandler.h"
#include "subsystems/hardware/leds_logic.h"

FunkyRobot::FunkyRobot() : GenericRobot{&container_} {
  RegisterPreference("num_coasting_loops", 1000);
  RegisterPreference("homing_flash_loops", 50);
}

void FunkyRobot::OnInitialize() {
  Field::Setup();

  // for (auto x : Field::getAllAutoData()) {
  //   Log("Adding Auto: {}", x.name + "_red");
  //   AddAuto(x.name + "_red", new GenericAuto{container_, x, false});
  //   Log("Adding Auto: {}", x.name + "_blue");
  //   AddAuto(x.name + "_blue", new GenericAuto{container_, x, true});
  // }

  ADD_AUTO_VARIANTS(FourAndPickAuto, "5PC");
  ADD_AUTO_VARIANTS(OnePieceAndNetAuto, "1PCN");
  ADD_AUTO_VARIANTS(LeaveAuto, "LEAVE");
  AddDefaultAuto("SIMTEST", new SimTestAuto{container_, false, true});

  // // Add dashboard buttons
  frc::SmartDashboard::PutData("set_cancoder_offsets",
      new frc846::wpilib::NTAction(
          [this] { container_.drivetrain_.SetCANCoderOffsets(); }));
  frc::SmartDashboard::PutData(
      "zero_bearing", new frc846::wpilib::NTAction(
                          [this] { container_.drivetrain_.ZeroBearing(); }));

  frc::SmartDashboard::PutData(
      "brake_for_time", new frc846::wpilib::NTAction([this] {
        container_.algal_ss_.elevator.CoastSubsystem();
        container_.coral_ss_.telescope.CoastSubsystem();
        container_.climber_.CoastSubsystem();
        container_.algal_ss_.algal_wrist.CoastSubsystem();
        container_.coral_ss_.coral_wrist.CoastSubsystem();
        coast_count_ = GetPreferenceValue_int("num_coasting_loops");
      }));

  frc::SmartDashboard::PutData(
      "home_telescope_elevator_climber", new frc846::wpilib::NTAction([this] {
        container_.coral_ss_.telescope.HomeSubsystem(
            robot_constants::telescope::min_height);
        container_.algal_ss_.elevator.HomeSubsystem(
            robot_constants::elevator::min_height_off_base);
        container_.climber_.ZeroClimber();
      }));

  frc::SmartDashboard::PutData("zero_coral_wrist",
      new frc846::wpilib::NTAction(
          [this] { container_.coral_ss_.coral_wrist.SetEncoderOffset(); }));

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
  if (!gyro_switch_.Get() && !IsEnabled()) {
    container_.drivetrain_.SetBearing(0_deg);
    homing_count_gyro = GetPreferenceValue_int("homing_flash_loops");
  }

  if (!home_switch_.Get() && !IsEnabled()) {
    container_.coral_ss_.telescope.HomeSubsystem(
        robot_constants::elevator::min_height_off_base);
    container_.algal_ss_.elevator.HomeSubsystem(
        robot_constants::telescope::min_height);
    container_.climber_.ZeroClimber();

    homing_count_ = GetPreferenceValue_int("homing_flash_loops");
  }

  if (container_.control_input_.GetReadings().home_elevator) {
    container_.algal_ss_.elevator.HomeSubsystem(
        robot_constants::elevator::min_height_off_base);

    homing_count_ = GetPreferenceValue_int("homing_flash_loops");
  }

  if (container_.control_input_.GetReadings().home_telescope) {
    container_.coral_ss_.telescope.HomeSubsystem(
        robot_constants::telescope::min_height);

    homing_count_ = GetPreferenceValue_int("homing_flash_loops");
  }

  if (coast_count_ > 0) coast_count_--;
  if (coast_count_ == 1 || coast_count_ == 7) {
    container_.algal_ss_.elevator.BrakeSubsystem();
    container_.coral_ss_.telescope.BrakeSubsystem();
    container_.climber_.BrakeSubsystem();
    container_.algal_ss_.algal_wrist.BrakeSubsystem();
    // container_.coral_ss_.coral_wrist.BrakeSubsystem();
  }
  if (!coast_switch_.Get() && !IsEnabled()) {
    container_.algal_ss_.elevator.CoastSubsystem();
    container_.coral_ss_.telescope.CoastSubsystem();
    container_.climber_.CoastSubsystem();
    container_.algal_ss_.algal_wrist.CoastSubsystem();
    container_.coral_ss_.coral_wrist.CoastSubsystem();
    coast_count_ = GetPreferenceValue_int("num_coasting_loops");
  }

  if (homing_count_ > 0) homing_count_--;
  if (homing_count_gyro > 0) homing_count_gyro--;

  bool isDisabled = frc::DriverStation::IsDisabled();

  if (homing_count_ > 0 && isDisabled)
    LEDsLogic::SetLEDsState(&container_, kLEDsHoming);
  else if (homing_count_gyro > 0 && isDisabled)
    LEDsLogic::SetLEDsState(&container_, kLEDsHomingGyro);
  else if (coast_count_ > 0 && isDisabled)
    LEDsLogic::CoastingLEDs(&container_,
        (1.0 * coast_count_) / GetPreferenceValue_int("num_coasting_loops"));
  else
    LEDsLogic::UpdateLEDs(&container_);

  AntiTippingCalculator::SetTelescopeHeight(
      container_.coral_ss_.telescope.GetReadings().position);
  AntiTippingCalculator::SetElevatorHeight(
      (container_.algal_ss_.elevator.GetReadings().position - 29_in) * 1.5 +
      29_in);

  auto cg = AntiTippingCalculator::CalculateRobotCG();
  Graph("robot_cg_x", cg[0]);
  Graph("robot_cg_y", cg[1]);
  Graph("robot_cg_z", cg[2]);
}

void FunkyRobot::InitTest() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});
  // container_.climber_.SetDefaultCommand(DinosaurClimberCommand{container_});

  // frc2::Trigger start_dinosaur_a([] { return true; });
  // start_dinosaur_a.WhileTrue(frc2::SequentialCommandGroup{
  //     AlgalPositionCommand{container_, kAlgae_DINOSAUR_A, true},
  //     frc2::WaitCommand{0.5_s},
  //     AlgalPositionCommand{container_, kAlgae_DINOSAUR_B, true},
  //     frc2::WaitCommand{0.5_s}}.Repeatedly());

  // frc2::Trigger start_dinosaur_c([] { return true; });
  // start_dinosaur_c.WhileTrue(frc2::SequentialCommandGroup{
  //     CoralPositionCommand{container_, kCoral_DINOSAUR_A, true},
  //     frc2::WaitCommand{0.5_s},
  //     CoralPositionCommand{container_, kCoral_DINOSAUR_B, true},
  //     frc2::WaitCommand{0.5_s}}.Repeatedly());
}

#ifndef RUNNING_FRC_TESTS
int main() {
  if (frc::RobotBase::IsSimulation()) configureSignalHandlers();
  return frc::StartRobot<FunkyRobot>();
}
#endif