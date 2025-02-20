// #pragma once

// #include "algal/algal_ss.h"
// #include "coral/coral_ss.h"
// #include "frc846/robot/GenericSubsystem.h"

// struct CoralgaeTarget {
//   CoralSSTarget coralTarget;
//   AlgalSSTarget algalTarget;
// };

// struct CoralgaeReadings {};

// class Coralgae
//     : public frc846::robot::GenericSubsystem<CoralgaeReadings,
//     CoralgaeTarget> {
// public:
//   Coralgae(AlgalSuperstructure* algae, CoralSuperstructure* coral);

//   // void HandleCommand(AlgalSSTarget algae_target);
//   // void HandleCommand(CoralSSTarget coral_target);

//   void Setup() override;

//   bool VerifyHardware() override;

//   CoralgaeTarget ZeroTarget() const override { return {}; };
//   CoralgaeReadings ReadFromHardware() override { return CoralgaeReadings{};
//   };

//   AlgalStates getLastAlgaeState();
//   CoralStates getLastCoralState();

//   void WriteToHardware(CoralgaeTarget target);

// private:
//   AlgalSuperstructure algal_ss;
//   CoralSuperstructure coral_ss;

//   // Coralgae algal_ss;
//   // Coralgae coral_ss;

//   bool is_algae_active;
//   bool is_coral_active;

//   CoralgaeTarget coralgaeTarget;
// };