#pragma once  

#include <units/angle.h>
#include <units/length.h>
#include <units/degree_t.h>


#include "frc/smartdashboard/Smartdashboard.h"
#include "frc846/ntinf/grapher.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/util/math.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include "ports.h"
#include "subsystems/hardware/drivetrain.h"

struct GPDTarget {};
struct GPDReadings {
  bool note_detected;
  units::degree_t note_angle;
  std::vector<frc846::util::Vector2D<units::foot_t>> points;
  int note_index;
  frc846::util::Vector2D<units::foot_t> closest_note;
};

class GPDSubsystem
    : public frc846::robot::GenericSubsystem<GPDReadings, GPDTarget> {
 public:
  GPDSubsystem(bool init);

  void Setup() override {};

  std::pair<frc846::util::Vector2D<units::foot_t>, int> getBestNote(
      const std::vector<frc846::util::Vector2D<units::foot_t>>& notes,
      const frc846::util::Vector2D<units::feet_per_second_t>& robot_velocity);

  frc846::util::Vector2D<units::foot_t> findDistance(units::degree_t theta_h,
                                                     units::degree_t theta_v);

  GPDTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  GPDReadings prevReadings;

  frc846::util::Position poseAtLastRequest;
  bool requested = false;
  int prevFrame = -1;

  frc846::base::Loggable readings_named{*this, "readings"};
  frc846::ntinf::Grapher<bool> note_detected_graph_{readings_named,
                                                    "note_detected"};

  frc846::base::Loggable algo_params{*this, "algo_parameters"};
  frc846::ntinf::Pref<units::foot_t> mount_height_{algo_params, "mount_height",
                                                   1_ft};
  frc846::ntinf::Pref<units::foot_t> note_height_{algo_params, "note_height",
                                                  0_ft};
  frc846::ntinf::Pref<units::second_t> nt_latency{algo_params, "nt_latency",
                                                  0_s};

  std::shared_ptr<nt::NetworkTable> gpdTable =
      nt::NetworkTableInstance::GetDefault().GetTable("gpd");
  frc846::ntinf::Grapher<units::foot_t> note_distance_magnitude_graph_{
      readings_named, "note_distance_magnitude_graph"};
  frc846::ntinf::Grapher<units::degree_t> note_angle_graph_{readings_named,
                                                            "note_angle_graph"};
  frc846::ntinf::Grapher<units::second_t> total_latency_{readings_named,
                                                         "total_latency_graph"};

  nt::NetworkTableInstance nt_table = nt::NetworkTableInstance::GetDefault();

  std::shared_ptr<nt::NetworkTable> raspiPreferences =
      nt::NetworkTableInstance::GetDefault().GetTable("RaspiPreferences");

  GPDReadings ReadFromHardware() override;

  void WriteToHardware(GPDTarget target) override;
};

public:
  GPDSubsystem();

  void Setup() override;

  ElevatorTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  bool WithinTolerance(units::inch_t pos) {
    return units::math::abs(pos - GetReadings().height) <
           GetPreferenceValue_unit_type<units::inch_t>("elevator_tolerance_");
  }

private:
  bool hasZeroed = false;

  // TODO: Set to correct reduction later
  elevator_pos_conv_t elevator_reduction_ = 1.0_in / 1.0_tr;

  frc846::control::config::MotorConstructionParameters motor_configs;
  frc846::control::HigherMotorController elevator_;
  frc846::control::HMCHelper<units::inch> motor_helper_;

  ElevatorReadings ReadFromHardware() override;

  void WriteToHardware(ElevatorTarget target) override;
};