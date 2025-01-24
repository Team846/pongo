#pragma once  

#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>


#include "frc/smartdashboard/Smartdashboard.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/math/constants.h"
#include "frc846/math/vectors.h"
#include "frc846/math/collection.h"
#include "frc846/base/Loggable.h"
#include "frc846/wpilib/NTAction.h"
#include "field.h"

#include "ports.h"

struct GPDTarget {};

struct GPDReadings {
  bool note_detected;
  units::degree_t note_angle;
  std::vector<frc846::math::Vector2D> notes;
  int note_index;
  frc846::math::Vector2D closest_note;
};

class GPDSubsystem
    : public frc846::robot::GenericSubsystem<GPDReadings, GPDTarget> {
 public:
  GPDSubsystem(bool init);

  std::pair<frc846::math::Vector2D, int> getBestNote(
      const std::vector<frc846::math::Vector2D>& notes,
      const frc846::math::Vector2D& robot_velocity);
  
  frc846::math::Vector2D findDistance(
      units::degree_t theta_h, units::degree_t theta_v);

  void Setup() override {};

  GPDTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  GPDReadings ReadFromHardware() override;

  void WriteToHardware(GPDTarget target) override;
  // bool WithinTolerance(units::inch_t pos) {}

private:
  Field g_field;
  GPDReadings prevReadings;

  frc846::base::Loggable readings_named{*this, "readings"};
  frc846::base::Loggable algo_params{"ago_parameters"};



  units::foot_t mount_height_ = GetPreferenceValue_unit_type<units::inch_t>("gpd_mount_height");
  units::foot_t algae_height_ = GetPreferenceValue_unit_type<units::inch_t>("gpd_algae_height");
  units::second_t latency_ = GetPreferenceValue_unit_type<units::second_t>("gpd_latency");
  
   std::shared_ptr<nt::NetworkTable> gpdTable =
      nt::NetworkTableInstance::GetDefault().GetTable("gpd");


};