// #pragma once

// #include "subsystems/hardware/algal/algal_ss.h"
// #include "subsystems/hardware/coral/coral_ss.h"

// using MonkeyState = std::pair<CoralStates, AlgalStates>;
// using CompositionMonkeyState = std::pair<std::pair<CoralStates, CoralStates>,
//     std::pair<AlgalStates, AlgalStates>>;
// using TransientMonkeyState =
//     std::pair<CompositionMonkeyState, CompositionMonkeyState>;

// using MSMAP = std::map<TransientMonkeyState, CompositionMonkeyState>;

// class FMS {
// public:
//   static CompositionMonkeyState convertToComposition(MonkeyState state);

//   static MSMAP intermediate_state_map;
// };