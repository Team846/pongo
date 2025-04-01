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
  REGISTER_SETPOINT("stow_no_piece", 29_in, 20_deg, -0.7);
  REGISTER_SETPOINT("flick", 29_in, 150_deg, -0.7);
  REGISTER_SETPOINT("stow_with_piece", 40_in, 150_deg, -0.15);
  REGISTER_SETPOINT("stow_net", 40_in, 90_deg, -0.15);
  REGISTER_SETPOINT("score_l2", 27.75_in, 190_deg, -0.15);
  REGISTER_SETPOINT("score_l3", 48.5_in, 190_deg, -0.15);
  REGISTER_SETPOINT("score_l4", 87_in, 220_deg, -0.15);

  REGISTER_SETPOINT("dinosaur_A", 33_in, 140_deg, -0.3);
  REGISTER_SETPOINT("dinosaur_B", 50_in, 200_deg, 0.3);

  RegisterPreference("score_dc", 1.0);

  RegisterPreference("init_telescope", true);
  RegisterPreference("init_wrist", true);
  RegisterPreference("init_ee", true);

  RegisterPreference("disable_distance_sensor", false);

  RegisterPreference("telescope_tolerance", 9_in);
  RegisterPreference("wrist_tolerance", 15_deg);

  RegisterPreference("telescope_adjustment", 0.1_in);
  RegisterPreference("wrist_adjustment", 0.7_deg);

  RegisterPreference("autostow", true);
  RegisterPreference("stow_no_piece_loop_thresh", 22);
  RegisterPreference("see_reef_loop_thresh", 3);

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
  case kCoral_FLICK: return GET_SETPOINT("flick");
  case kCoral_StowWithPiece: return GET_SETPOINT("stow_with_piece");
  case kCoral_StowNet: return GET_SETPOINT("stow_net");
  case kCoral_ScoreL2: return GET_SETPOINT("score_l2");
  case kCoral_ScoreL3: return GET_SETPOINT("score_l3");
  case kCoral_ScoreL4: return GET_SETPOINT("score_l4");
  case kCoral_DINOSAUR_A: return GET_SETPOINT("dinosaur_A");
  case kCoral_DINOSAUR_B: return GET_SETPOINT("dinosaur_B");
  default: return GET_SETPOINT("stow_no_piece");
  }
}

bool CoralSuperstructure::hasReached(CoralStates state) {
  bool has_reached = hasReachedTelescope(state) && hasReachedWrist(state);
  Graph("has_reached", has_reached);
  return has_reached;
}

bool CoralSuperstructure::hasReachedTelescope(CoralStates state) {
  if (!telescope.is_initialized()) return true;

  CoralSetpoint setpoint = getSetpoint(state);

  return (units::math::abs(telescope.GetReadings().position - setpoint.height) <
          GetPreferenceValue_unit_type<units::inch_t>("telescope_tolerance"));
}

bool CoralSuperstructure::hasReachedWrist(CoralStates state) {
  if (!coral_wrist.is_initialized()) return true;

  CoralSetpoint setpoint = getSetpoint(state);

  return (
      units::math::abs(coral_wrist.GetReadings().position - setpoint.angle) <
      GetPreferenceValue_unit_type<units::degree_t>("wrist_tolerance"));
}

CoralSSReadings CoralSuperstructure::ReadFromHardware() {
  telescope.UpdateReadings();
  coral_wrist.UpdateReadings();
  coral_end_effector.UpdateReadings();

  if (coral_end_effector.GetReadings().has_piece_) {
    no_piece_count_ = 0;

  } else {
    no_piece_count_++;
  }
  Graph("no_piece_count", no_piece_count_);

  bool autostow_valid =
      GetPreferenceValue_bool("autostow") &&
      (no_piece_count_ > GetPreferenceValue_int("stow_no_piece_loop_thresh"));
  Graph("autostow_valid", autostow_valid);

  bool chute_piece = !chute_sensor_.Get();
  Graph("chute_piece", chute_piece);

  bool piece_entered =
      chute_piece || coral_end_effector.GetReadings().has_piece_;
  Graph("piece_entered", piece_entered);

  return {autostow_valid, piece_entered};
}

void CoralSuperstructure::WriteToHardware(CoralSSTarget target) {
  CoralSetpoint setpoint = getSetpoint(target.state);

  if (target.state != last_state) {
    clearAdjustments();
    if (hasReached(target.state)) last_state = target.state;
  }

  if (last_state == CoralStates::kCoral_StowNoPiece ||
      last_state == CoralStates::kCoral_StowWithPiece ||
      last_state == CoralStates::kCoral_StowNet) {
    telescope.SetTarget({setpoint.height + telescope_adjustment_});

    if (hasReachedTelescope(target.state)) {
      coral_wrist.SetTarget({setpoint.angle + wrist_adjustment_});
    }
  } else if ((last_state == kCoral_ScoreL2 || last_state == kCoral_ScoreL3 ||
                 last_state == kCoral_ScoreL4) &&
             (target.state == kCoral_ScoreL2 ||
                 target.state == kCoral_ScoreL3 ||
                 target.state == kCoral_ScoreL4) &&
             (last_state != target.state)) {
    coral_wrist.SetTarget({getSetpoint(kCoral_StowWithPiece).angle});

    if (hasReachedWrist(kCoral_StowWithPiece)) {
      last_state = kCoral_StowWithPiece;
    }
  } else {
    coral_wrist.SetTarget({setpoint.angle + wrist_adjustment_});

    if (hasReachedWrist(target.state)) {
      telescope.SetTarget({setpoint.height + telescope_adjustment_});
    }
  }

  if (coral_end_effector.GetReadings().see_reef)
    see_reef_count_ = GetPreferenceValue_int("see_reef_loop_thresh");
  else if (see_reef_count_ > 0)
    see_reef_count_ -= 1;

  if (target.score ||
      (see_reef_count_ > 0 &&
          (target.state == kCoral_ScoreL2 || target.state == kCoral_ScoreL3 ||
              target.state == kCoral_ScoreL4) &&
          hasReached(target.state))) {
    Graph("auto_score", true);
    coral_end_effector.SetTarget({GetPreferenceValue_double("score_dc")});
  } else {
    Graph("auto_score", false);
    coral_end_effector.SetTarget({setpoint.ee_dc});
  }

  telescope.UpdateHardware();
  coral_wrist.UpdateHardware();
  coral_end_effector.UpdateHardware();
}

void CoralSuperstructure::adjustTelescope(bool upwards) {
  telescope_adjustment_ +=
      (upwards ? 1 : -1) *
      GetPreferenceValue_unit_type<units::inch_t>("telescope_adjustment");
}

void CoralSuperstructure::adjustWrist(bool upwards) {
  wrist_adjustment_ +=
      (upwards ? 1 : -1) *
      GetPreferenceValue_unit_type<units::degree_t>("wrist_adjustment");
}

void CoralSuperstructure::clearAdjustments() {
  telescope_adjustment_ = 0_in;
  wrist_adjustment_ = 0_deg;
}
