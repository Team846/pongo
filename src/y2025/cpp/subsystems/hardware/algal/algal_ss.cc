#include "subsystems/hardware/algal/algal_ss.h"

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

AlgalSuperstructure::AlgalSuperstructure()
    : GenericSubsystem("algal_ss"),
      elevator(),
      algal_wrist(),
      algal_end_effector() {
  REGISTER_SETPOINT("stow", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("processor", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("ground_intake", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("on_top_intake", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("net", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("l2_pick", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("l3_pick", 0_in, 0_deg, 0.0);

  REGISTER_SETPOINT("dinosaur_A", 0_in, 0_deg, 0.0);
  REGISTER_SETPOINT("dinosaur_B", 0_in, 0_deg, 0.0);

  RegisterPreference("score_dc", -0.5);

  RegisterPreference("init_elevator", false);
  RegisterPreference("init_wrist", false);
  RegisterPreference("init_ee", false);

  RegisterPreference("elevator_tolerance", 3_in);
  RegisterPreference("wrist_tolerance", 7_deg);
}

void AlgalSuperstructure::Setup() {
  if (GetPreferenceValue_bool("init_elevator")) {
    elevator.Init();
    elevator.Setup();
  }

  if (GetPreferenceValue_bool("init_wrist")) {
    algal_wrist.Init();
    algal_wrist.Setup();
  }

  if (GetPreferenceValue_bool("init_ee")) {
    algal_end_effector.Init();
    algal_end_effector.Setup();
  }
}

bool AlgalSuperstructure::VerifyHardware() {
  return elevator.VerifyHardware() && algal_wrist.VerifyHardware() &&
         algal_end_effector.VerifyHardware();
}

AlgalSetpoint AlgalSuperstructure::getSetpoint(AlgalStates state) {
  switch (state) {
  case kAlgae_Stow: return GET_SETPOINT("stow");
  case kAlgae_Processor: return GET_SETPOINT("processor");
  case kAlgae_GroundIntake: return GET_SETPOINT("ground_intake");
  case kAlgae_OnTopIntake: return GET_SETPOINT("on_top_intake");
  case kAlgae_Net: return GET_SETPOINT("net");
  case kAlgae_L2Pick: return GET_SETPOINT("l2_pick");
  case kAlgae_L3Pick: return GET_SETPOINT("l3_pick");
  default: return GET_SETPOINT("stow");
  }
}

bool AlgalSuperstructure::hasReached(AlgalStates state) {
  AlgalSetpoint setpoint = getSetpoint(state);

  if (units::math::abs(elevator.GetReadings().position - setpoint.height) >
      GetPreferenceValue_unit_type<units::inch_t>("elevator_tolerance"))
    return false;

  if (units::math::abs(algal_wrist.GetReadings().position - setpoint.angle) >
      GetPreferenceValue_unit_type<units::degree_t>("wrist_tolerance"))
    return false;

  return true;
}

AlgalSSReadings AlgalSuperstructure::ReadFromHardware() {
  elevator.UpdateReadings();
  algal_wrist.UpdateReadings();
  algal_end_effector.UpdateReadings();

  return {};
}

void AlgalSuperstructure::WriteToHardware(AlgalSSTarget target) {
  AlgalSetpoint setpoint = getSetpoint(target.state);

  elevator.SetTarget({setpoint.height});

  if (target.separate_wrist_state.has_value())
    algal_wrist.SetTarget(
        {getSetpoint(target.separate_wrist_state.value()).angle});
  else
    algal_wrist.SetTarget({setpoint.angle});

  if (target.score)
    algal_end_effector.SetTarget({GetPreferenceValue_double("score_dc")});
  else
    algal_end_effector.SetTarget({setpoint.ee_dc});

  elevator.UpdateHardware();
  algal_wrist.UpdateHardware();
  algal_end_effector.UpdateHardware();
}
