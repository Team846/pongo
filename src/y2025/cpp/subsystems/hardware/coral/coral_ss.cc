#include "subsystems/hardware/coral/coral_ss.h"

#define REGISTER_SETPOINT(name, height, angle, ee_dc)                       \
  RegisterPreference(std::string("setpoints/") + name + "/height", height); \
  RegisterPreference(std::string("setpoints/") + name + "/angle", angle);   \
  RegisterPreference(std::string("setpoints/") + name + "/ee_dc", ee_dc);

#define GET_SETPOINT(name)                                    \
  {.height = GetPreferenceValue_unit_type<units::inch_t>(     \
       std::string("setpoints/") + name + "/height"),         \
      .angle = GetPreferenceValue_unit_type<units::degree_t>( \
          std::string("setpoints/") + name + "/angle"),       \
      .ee_dc = GetPreferenceValue_double(                     \
          std::string("setpoints/") + name + "/ee_dc")}

CoralSuperstructure::CoralSuperstructure()
    : GenericSubsystem("coral_ss"),
      telescope(),
      coral_wrist(),
      coral_end_effector() {
  REGISTER_SETPOINT("stow_no_piece", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("stow_with_piece", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("score_l2", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("score_l3", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("score_l4", 0_in, 0_deg, 0.0);

  REGISTER_SETPOINT("dinosaur_A", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("dinosaur_B", 0_in, 0_deg, 0.0);

  RegisterPreference("score_dc", -0.5);

  RegisterPreference("init_telescope", false);
  RegisterPreference("init_wrist", false);
  RegisterPreference("init_ee", true);

  RegisterPreference("disable_distance_sensor", false);

  RegisterPreference("telescope_tolerance", 1.2_in);
  RegisterPreference("wrist_tolerance", 5_deg);

  last_state = kCoral_StowNoPiece;
}

void CoralSuperstructure::Setup() {
  if (GetPreferenceValue_bool("init_telescope")) {
    telescope.Init();
    telescope.Setup();
  }
  if (GetPreferenceValue_bool("init_wrist")) {
    coral_wrist.Init();
    coral_wrist.Setup();
  }

  if (GetPreferenceValue_bool("init_ee")) {
    coral_end_effector.Init();
    coral_end_effector.Setup();
  }
}

bool CoralSuperstructure::VerifyHardware() {
  return telescope.VerifyHardware() && coral_wrist.VerifyHardware() &&
         coral_end_effector.VerifyHardware();
}

CoralSetpoint CoralSuperstructure::getSetpoint(CoralStates state) {
  switch (state) {
  case kCoral_StowNoPiece: return GET_SETPOINT("stow_no_piece");
  case kCoral_StowWithPiece: return GET_SETPOINT("stow_with_piece");
  case kCoral_ScoreL2: return GET_SETPOINT("score_l2");
  case kCoral_ScoreL3: return GET_SETPOINT("score_l3");
  case kCoral_ScoreL4: return GET_SETPOINT("score_l4");
  case kCoral_DINOSAUR_A: return GET_SETPOINT("dinosaur_A");
  case kCoral_DINOSAUR_B: return GET_SETPOINT("dinosaur_B");
  default: return GET_SETPOINT("stow_no_piece");
  }
}

bool CoralSuperstructure::hasReached(CoralStates state) {
  return hasReachedTelescope(state) && hasReachedWrist(state);
}

bool CoralSuperstructure::hasReachedTelescope(CoralStates state) {
  CoralSetpoint setpoint = getSetpoint(state);

  return (units::math::abs(telescope.GetReadings().position - setpoint.height) <
          GetPreferenceValue_unit_type<units::inch_t>("telescope_tolerance"));
}

bool CoralSuperstructure::hasReachedWrist(CoralStates state) {
  CoralSetpoint setpoint = getSetpoint(state);

  return (
      units::math::abs(coral_wrist.GetReadings().position - setpoint.angle) <
      GetPreferenceValue_unit_type<units::degree_t>("wrist_tolerance"));
}

CoralSSReadings CoralSuperstructure::ReadFromHardware() {
  telescope.UpdateReadings();
  coral_wrist.UpdateReadings();
  coral_end_effector.UpdateReadings();

  return {};
}

void CoralSuperstructure::WriteToHardware(CoralSSTarget target) {
  CoralSetpoint setpoint = getSetpoint(target.state);

  // change last_state if you've reached that state
  if (target.state != last_state) {
    if (hasReached(target.state)) last_state = target.state;
  }

  if (last_state == CoralStates::kCoral_StowWithPiece) {
    // if at stow with piece, move telescope first
    telescope.SetTarget({setpoint.height});

    if (hasReachedTelescope(target.state)) {
      coral_wrist.SetTarget({setpoint.angle});
    }
  }
  // if you're currently placing, and you want to change levels
  //'change mind coral'
  else if ((last_state == kCoral_ScoreL2 || last_state == kCoral_ScoreL3 ||
               last_state == kCoral_ScoreL4) &&
           (target.state == kCoral_ScoreL2 || target.state == kCoral_ScoreL3 ||
               target.state == kCoral_ScoreL4) &&
           (last_state != target.state)) {
    coral_wrist.SetTarget({getSetpoint(kCoral_StowWithPiece).angle});

    if (hasReachedWrist(kCoral_StowWithPiece)) {
      last_state = kCoral_StowWithPiece;  // fake stow with piece
    }
  }
  // if going to stow or holding position
  else {
    coral_wrist.SetTarget({setpoint.angle});

    if (hasReachedTelescope(target.state)) {
      telescope.SetTarget({setpoint.height});
    }
  }

  if (target.score ||
      (!GetPreferenceValue_bool("disable_distance_sensor") &&
          coral_end_effector.GetReadings().has_piece_ &&
          coral_end_effector.GetReadings().see_reef &&
          (target.state == kCoral_ScoreL2 || target.state == kCoral_ScoreL3 ||
              target.state == kCoral_ScoreL4) &&
          hasReached(target.state)))
    coral_end_effector.SetTarget({GetPreferenceValue_double("score_dc")});
  else
    coral_end_effector.SetTarget({setpoint.ee_dc});

  telescope.UpdateHardware();
  coral_wrist.UpdateHardware();
  coral_end_effector.UpdateHardware();
}
