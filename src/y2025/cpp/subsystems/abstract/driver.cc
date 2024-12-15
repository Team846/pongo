#include "subsystems/abstract/driver.h"

#include "frc846/util/share_tables.h"

DriverSubsystem::DriverSubsystem()
    : frc846::robot::GenericSubsystem<DriverReadings, DriverTarget>{"driver",
                                                                    true} {}

DriverTarget DriverSubsystem::ZeroTarget() const {
  DriverTarget target;
  target.rumble = false;
  return target;
}

bool DriverSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(xbox_.IsConnected(), ok, "not connected");
  return ok;
}

DriverReadings DriverSubsystem::ReadFromHardware() {
  DriverReadings readings{xbox_, trigger_threshold_.value()};

  previous_readings_ = readings;

  return readings;
}

void DriverSubsystem::WriteToHardware(DriverTarget target) {
  target_rumble_graph_.Graph(target.rumble);

  auto rumble = (target.rumble) ? rumble_strength_.value() : 0;

  xbox_.SetRumble(frc::XboxController::RumbleType::kLeftRumble, rumble);
  xbox_.SetRumble(frc::XboxController::RumbleType::kRightRumble, rumble);
}