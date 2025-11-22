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
// #include "calculators/AntiTippingCalculator.h"
#include "commands/teleop/drive_command.h"
#include "control_triggers.h"
#include "frc846/wpilib/NTAction.h"
#include "rsighandler.h"
#include "subsystems/hardware/leds_logic.h"

FunkyRobot::FunkyRobot() : GenericRobot{&container_} {
  RegisterPreference("num_coasting_loops", 1000);
  RegisterPreference("homing_flash_loops", 50);

  // std::thread visionThread{[&]() {
  //   VisionThread(&container_);
  // }};
  // visionThread.detach();
}

void FunkyRobot::OnInitialize() {
  // Add dashboard buttons
  frc::SmartDashboard::PutData("set_cancoder_offsets",
      new frc846::wpilib::NTAction(
          [this] { container_.drivetrain_.SetCANCoderOffsets(); }));
  frc::SmartDashboard::PutData(
      "zero_bearing", new frc846::wpilib::NTAction(
                          [this] { container_.drivetrain_.ZeroBearing(); }));

  frc::SmartDashboard::PutData("zero_odometry",
      new frc846::wpilib::NTAction(
          [this] { container_.drivetrain_.SetPosition({0_in, 0_in}); }));

  // Add path recording controls
  frc::SmartDashboard::PutData(
      "start_path_recording", new frc846::wpilib::NTAction([this] {
        auto timestamp =
            std::chrono::system_clock::now().time_since_epoch().count();
        std::string filename = "path_" + std::to_string(timestamp);
        container_.drivetrain_.StartPathRecording(filename);
        Log("Started recording path data to {}.csv", filename);
      }));

  frc::SmartDashboard::PutData(
      "stop_path_recording", new frc846::wpilib::NTAction([this] {
        bool success = container_.drivetrain_.StopPathRecording();
        if (success) {
          Log("Successfully stopped recording path data");
        } else {
          Warn(
              "Failed to stop recording path data or no recording in progress");
        }
      }));
}

void FunkyRobot::OnEnable() {
  // Start path recording

  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);

  // Format time as month-day-year-hour-minute-second
  std::tm* tm_now = std::localtime(&time_t_now);

  char time_buffer[64];
  std::strftime(time_buffer, sizeof(time_buffer), "%m-%d-%Y_%H-%M-%S", tm_now);
  std::string filename = "pathlogs_" + std::string(time_buffer);

  container_.drivetrain_.StartPathRecording(filename);
  Log("Started recording auto path data to {}.csv", filename);
}

void FunkyRobot::OnDisable() {
  // Stop path recording
  bool success = container_.drivetrain_.StopPathRecording();
  if (success) {
    Log("Successfully stopped recording path data");
  } else {
    Warn("Failed to stop recording path data or no recording in progress");
  }
}

void FunkyRobot::InitTeleop() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});

  ControlTriggerInitializer::InitTeleopTriggers(container_);
}

void FunkyRobot::OnPeriodic() {
  if (!gyro_switch_.Get() && !IsEnabled()) {
    container_.drivetrain_.SetBearing(0_deg);
    homing_count_gyro = GetPreferenceValue_int("homing_flash_loops");
  }

  if (!home_switch_.Get() && !IsEnabled()) {
    homing_count_ = GetPreferenceValue_int("homing_flash_loops");
  }

  if (coast_count_ > 0) coast_count_--;
  if (coast_count_ == 1 || coast_count_ == 7) {}
  if (!coast_switch_.Get() && !IsEnabled()) {
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

  // AntiTippingCalculator::SetTelescopeHeight(
  //     container_.coral_ss_.telescope.GetReadings().position);
  // AntiTippingCalculator::SetElevatorHeight(
  //     (container_.algal_ss_.elevator.GetReadings().position - 29_in) * 1.5 +
  //     29_in);

  // auto cg = AntiTippingCalculator::CalculateRobotCG();
  // Graph("robot_cg_x", cg[0]);
  // Graph("robot_cg_y", cg[1]);
  // Graph("robot_cg_z", cg[2]);
}

void FunkyRobot::InitTest() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});
}

#ifndef RUNNING_FRC_TESTS
int main() {
  if (frc::RobotBase::IsSimulation()) configureSignalHandlers();
  return frc::StartRobot<FunkyRobot>();
}
#endif