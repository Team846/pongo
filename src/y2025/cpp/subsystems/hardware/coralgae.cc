// #include "subsystems/hardware/coralgae.h"

// Coralgae::Coralgae(AlgalSuperstructure* algae, CoralSuperstructure* coral)
//     : frc846::robot::GenericSubsystem<CoralgaeReadings,
//           CoralgaeTarget>{"coralgae"},
//       algal_ss{},
//       coral_ss{} {}

// void Coralgae::Setup() {}

// bool Coralgae::VerifyHardware() {
//   return (coral_ss.VerifyHardware() && coral_ss.coral_wrist.VerifyHardware()
//   &&
//              coral_ss.coral_end_effector.VerifyHardware()) &&
//          (algal_ss.elevator.VerifyHardware() &&
//              algal_ss.algal_wrist.VerifyHardware() &&
//              algal_ss.algal_end_effector.VerifyHardware());
// }

// void Coralgae::WriteToHardware(CoralgaeTarget target) {
//   if (target.algalTarget.state == AlgalStates::kAlgae_L2Pick &&
//       target.coralTarget.state == CoralStates::kCoral_ScoreL2) {
//     if (algal_ss.last_state == kAlgae_L2Pick) {
//       algal_ss.SetTarget(target.algalTarget);
//     } else {
//       coral_ss.SetTarget(target.coralTarget);
//     }
//   } else if (target.algalTarget.state == AlgalStates::kAlgae_L2Pick &&
//              target.coralTarget.state == CoralStates::kCoral_ScoreL3) {
//     if (algal_ss.last_state == kAlgae_L2Pick) {
//       algal_ss.SetTarget(target.algalTarget);
//     } else {
//       coral_ss.SetTarget(target.coralTarget);
//     }
//   } else if (target.algalTarget.state == AlgalStates::kAlgae_L3Pick &&
//              target.coralTarget.state == CoralStates::kCoral_ScoreL3) {
//     if (algal_ss.last_state == kAlgae_L3Pick) {
//       algal_ss.SetTarget(target.algalTarget);
//     } else {
//       coral_ss.SetTarget(target.coralTarget);
//     }
//   } else {
//     algal_ss.SetTarget(target.algalTarget);
//     coral_ss.SetTarget(target.coralTarget);
//   }
// }