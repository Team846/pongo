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

  RegisterPreference("score_dc", -0.5);

  RegisterPreference("init_telescope", false);
  RegisterPreference("init_wrist", false);
  RegisterPreference("init_ee", true);
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
  default: return GET_SETPOINT("stow_no_piece");
  }
}

CoralSSReadings CoralSuperstructure::ReadFromHardware() {
  telescope.UpdateReadings();
  coral_wrist.UpdateReadings();
  coral_end_effector.UpdateReadings();

  return {};
}

void CoralSuperstructure::WriteToHardware(CoralSSTarget target) {
  CoralSetpoint setpoint = getSetpoint(target.state);

  telescope.SetTarget({setpoint.height});
  coral_wrist.SetTarget({setpoint.angle});

  // TODO: Check setpoint state of each subsystem
  if (target.score || (!GetPreferenceValue_bool("Override_distance_sensor") &&
                          coral_end_effector.GetReadings().has_piece_))
    coral_end_effector.SetTarget({GetPreferenceValue_double("score_dc")});
  else
    coral_end_effector.SetTarget({setpoint.ee_dc});

  telescope.UpdateHardware();
  coral_wrist.UpdateHardware();
  coral_end_effector.UpdateHardware();
}
