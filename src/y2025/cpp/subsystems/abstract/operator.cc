#include "subsystems/abstract/operator.h"

#include "frc846/util/share_tables.h"

OperatorSubsystem::OperatorSubsystem()
    : frc846::robot::GenericSubsystem<OperatorReadings, OperatorTarget>{
          "operator", true} {}

OperatorTarget OperatorSubsystem::ZeroTarget() const {
  OperatorTarget target;
  target.rumble = false;
  return target;
}

bool OperatorSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(xbox_.IsConnected(), ok, "not connected");
  return ok;
}

OperatorReadings OperatorSubsystem::ReadFromHardware() {
  OperatorReadings readings{xbox_, trigger_threshold_.value()};

  previous_readings_ = readings;

  return readings;
}

void OperatorSubsystem::WriteToHardware(OperatorTarget target) {
  target_rumble_graph_.Graph(target.rumble);

  auto rumble = target.rumble || frc846::util::ShareTables::GetBoolean(
                                     "climb_hooks_engaged")
                    ? rumble_strength_.value()
                    : 0;

  xbox_.SetRumble(frc::XboxController::RumbleType::kLeftRumble, rumble);
  xbox_.SetRumble(frc::XboxController::RumbleType::kRightRumble, rumble);
}