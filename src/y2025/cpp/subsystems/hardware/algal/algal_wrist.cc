#include "subsystems/hardware/algal/algal_wrist.h"

#include "ports.h"

// AlgalWristSubsystem::AlgalWristSubsystem()
//     : WristSubsystem("algal_wrist",
//           frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO,
//           frc846::control::config::MotorConstructionParameters{
//               .can_id = ports::algal_ss_::wrist_::kWristMotor_CANID,
//               . = frc846::control::base::MotorType::kBrushless,
//               .inverted = false,
//               .sensor_phase = true,
//               .current_limit = 20_A,
//               .brake_mode = true,
//           },
//           units::degree_t{1.0 / 360.0}) {}

void AlgalWristSubsystem::ExtendedSetup() {
  // TODO: implement
}

std::pair<units::degree_t, bool> AlgalWristSubsystem::GetSensorPos() {
  // TODO: implement
  return {0_deg, false};
}