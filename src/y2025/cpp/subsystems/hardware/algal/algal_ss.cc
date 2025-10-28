#include "subsystems/hardware/algal/algal_ss.h"

#define REGISTER_SETPOINT(name, height, angle, ee_vel)                      \
  RegisterPreference(std::string("setpoints/") + name + "/height", height); \
  RegisterPreference(std::string("setpoints/") + name + "/angle", angle);   \
  RegisterPreference(std::string("setpoints/") + name + "/ee_vel", ee_vel);

#define GET_SETPOINT(name)                                              \
  {.height = GetPreferenceValue_unit_type<units::inch_t>(               \
       std::string("setpoints/") + name + "/height"),                   \
      .angle = GetPreferenceValue_unit_type<units::degree_t>(           \
          std::string("setpoints/") + name + "/angle"),                 \
      .ee_vel = GetPreferenceValue_unit_type<units::feet_per_second_t>( \
          std::string("setpoints/") + name + "/ee_vel")}

AlgalSuperstructure::AlgalSuperstructure()
    : GenericSubsystem("algal_ss"),
      elevator(),
      algal_wrist(),
      algal_end_effector() {
  REGISTER_SETPOINT("stow", 30_in, 0_deg, 9.2_fps);
  REGISTER_SETPOINT("processor", 29_in, 25_deg, 18.35_fps);
  REGISTER_SETPOINT("ground_intake", 29_in, 58_deg, 82.5_fps);
  REGISTER_SETPOINT("on_top_intake", 35_in, 58_deg, 82.5_fps);
  REGISTER_SETPOINT("net", 72_in, 0_deg, 18.35_fps);
  REGISTER_SETPOINT("net_inter", 35_in, 40_deg, 9.2_fps);
  REGISTER_SETPOINT("l2_pick", 41.5_in, 30_deg, 64.15_fps);
  REGISTER_SETPOINT("l3_pick", 50_in, 30_deg, 64.15_fps);

  REGISTER_SETPOINT("coral_pick", 34_in, 105_deg, -64.15_fps);
  REGISTER_SETPOINT("l1_coral_score", 34_in, 105_deg, -9.2_fps);

  REGISTER_SETPOINT("dinosaur_A", 34_in, 0_deg, -27.5);
  REGISTER_SETPOINT("dinosaur_B", 45_in, 35_deg, 27.5);

  RegisterPreference("score_dc", -16.5_fps);

  RegisterPreference("score_coral_vel", -16.5_fps);

  RegisterPreference("init_elevator", true);
  RegisterPreference("init_wrist", true);
  RegisterPreference("init_ee", true);

  RegisterPreference("elevator_tolerance", 5_in);
  RegisterPreference("wrist_tolerance", 12_deg);

  RegisterPreference("elevator_adjustment", 0.04_in);
  RegisterPreference("wrist_adjustment", 0.2_deg);
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
  case kAlgae_NetInter: return GET_SETPOINT("net_inter");
  case kAlgae_L2Pick: return GET_SETPOINT("l2_pick");
  case kAlgae_L3Pick: return GET_SETPOINT("l3_pick");
  case kAlgae_CoralPick: return GET_SETPOINT("coral_pick");
  case kAlgae_L1CoralScore: return GET_SETPOINT("l1_coral_score");
  default: return GET_SETPOINT("stow");
  }
}

bool AlgalSuperstructure::hasReached(AlgalStates state) {
  bool has_reached = hasReachedWrist(state) && hasReachedElevator(state);

  // Graph("has_reached", has_reached);

  return has_reached;
}

bool AlgalSuperstructure::hasReachedWrist(AlgalStates state) {
  AlgalSetpoint setpoint = getSetpoint(state);

  if (algal_wrist.is_initialized() &&
      (units::math::abs(algal_wrist.GetReadings().position - setpoint.angle) >
          GetPreferenceValue_unit_type<units::degree_t>("wrist_tolerance"))) {
    // Graph("has_reached", false);
    return false;
  }
  return true;
}

bool AlgalSuperstructure::hasReachedElevator(AlgalStates state) {
  AlgalSetpoint setpoint = getSetpoint(state);

  if (elevator.is_initialized() &&
      (units::math::abs(elevator.GetReadings().position - setpoint.height) >
          GetPreferenceValue_unit_type<units::inch_t>("elevator_tolerance"))) {
    // Graph("has_reached", false);
    return false;
  }
  return true;
}

AlgalSSReadings AlgalSuperstructure::ReadFromHardware() {
  elevator.UpdateReadings();
  algal_wrist.UpdateReadings();
  algal_end_effector.UpdateReadings();

  return {algal_end_effector.GetReadings().has_piece_};
}

void AlgalSuperstructure::WriteToHardware(AlgalSSTarget target) {
  AlgalSetpoint setpoint = getSetpoint(target.state);
  bool coral_mode = target.cm;

  Graph("is_coral_mode", coral_mode);

  if (target.super_mode) {
    algal_end_effector.SetTarget(
        {GetPreferenceValue_unit_type<units::feet_per_second_t>("score_dc"),
            true, false, false, true});
  }

  if (target.score) {
    if (coral_mode) {
      algal_end_effector.ClearHasPiece();
      algal_end_effector.SetTarget(
          {GetPreferenceValue_unit_type<units::feet_per_second_t>(
               "score_coral_vel"),
              false});
    } else if (target.state == kAlgae_Processor) {
      algal_end_effector.SetTarget(
          {GetPreferenceValue_unit_type<units::feet_per_second_t>("score_dc")});
    } else {
      algal_end_effector.SetTarget(
          {GetPreferenceValue_unit_type<units::feet_per_second_t>("score_dc"),
              true, false, false, target.super_mode});
    }
  } else if (coral_mode)
    algal_end_effector.SetTarget(
        {setpoint.ee_vel, false, target.cm, target.state != kAlgae_CoralPick});
  else
    algal_end_effector.SetTarget({setpoint.ee_vel});

  if (target.state != last_state) {
    clearAdjustments();
    if (hasReached(target.state)) last_state = target.state;
  }

  bool lastIsHigh = last_state == AlgalStates::kAlgae_L2Pick ||
                    last_state == AlgalStates::kAlgae_L3Pick ||
                    last_state == AlgalStates::kAlgae_Net ||
                    last_state == AlgalStates::kAlgae_GroundIntake ||
                    last_state == AlgalStates::kAlgae_OnTopIntake ||
                    last_state == AlgalStates::kAlgae_CoralPick ||
                    last_state == AlgalStates::kAlgae_L1CoralScore;
  bool targetIsHigh = target.state == AlgalStates::kAlgae_L2Pick ||
                      target.state == AlgalStates::kAlgae_L3Pick ||
                      target.state == AlgalStates::kAlgae_Net ||
                      target.state == AlgalStates::kAlgae_GroundIntake ||
                      target.state == AlgalStates::kAlgae_OnTopIntake ||
                      target.state == AlgalStates::kAlgae_CoralPick ||
                      target.state == AlgalStates::kAlgae_L1CoralScore;

  if (last_state == AlgalStates::kAlgae_Stow) {
    elevator.SetTarget({setpoint.height + elevator_adjustment_});
    if (hasReachedElevator(target.state))
      algal_wrist.SetTarget({setpoint.angle + wrist_adjustment_});
  } else if (lastIsHigh && (target.state == AlgalStates::kAlgae_Stow)) {
    algal_wrist.SetTarget({setpoint.angle + wrist_adjustment_});
    if (hasReachedWrist(target.state))
      elevator.SetTarget({setpoint.height + elevator_adjustment_});
  } else if ((target.state == kAlgae_GroundIntake &&
                 last_state == kAlgae_OnTopIntake) ||
             (last_state == kAlgae_GroundIntake &&
                 target.state == kAlgae_OnTopIntake)) {
    elevator.SetTarget({setpoint.height + elevator_adjustment_});
    algal_wrist.SetTarget({setpoint.angle + wrist_adjustment_});
  } else if (lastIsHigh && targetIsHigh && last_state != target.state) {
    algal_wrist.SetTarget({getSetpoint(AlgalStates::kAlgae_Stow).angle});
    if (hasReachedWrist(AlgalStates::kAlgae_Stow)) {
      last_state = AlgalStates::kAlgae_Stow;
      elevator.SetTarget({setpoint.height + elevator_adjustment_});
    }
  } else {
    elevator.SetTarget({setpoint.height + elevator_adjustment_});
    algal_wrist.SetTarget({setpoint.angle + wrist_adjustment_});
  }

  elevator.UpdateHardware();
  algal_wrist.UpdateHardware();
  algal_end_effector.UpdateHardware();
}

void AlgalSuperstructure::adjustElevator(bool upwards) {
  elevator_adjustment_ +=
      (upwards ? 1 : -1) *
      GetPreferenceValue_unit_type<units::inch_t>("elevator_adjustment");
}

void AlgalSuperstructure::adjustWrist(bool upwards) {
  wrist_adjustment_ +=
      (upwards ? 1 : -1) *
      GetPreferenceValue_unit_type<units::degree_t>("wrist_adjustment");
}

void AlgalSuperstructure::clearAdjustments() {
  elevator_adjustment_ = 0_in;
  wrist_adjustment_ = 0_deg;
}
