#include "reef.h"

#include <frc/DriverStation.h>

#include "field.h"

frc846::math::FieldPoint ReefProvider::reefPoint = {
    {158.5_in, 144_in + 32.75_in}, 0_deg, 0_fps};

std::vector<frc846::math::FieldPoint> ReefProvider::getReefScoringLocations(
    bool mirrorIfBlue) {
  std::vector<frc846::math::FieldPoint> reefScoringLocations;

  frc846::math::Vector2D reef_center = reefPoint.point;
  frc846::math::Vector2D left_reef_displacement =
      frc846::math::Vector2D{6.5_in, 49.5_in};

  frc846::math::Vector2D right_reef_displacement =
      frc846::math::Vector2D{-6.5_in, 49.5_in};

  bool mirror = mirrorIfBlue && (frc::DriverStation::GetAlliance() ==
                                    frc::DriverStation::kBlue);

  for (int i = 0; i < 6; i++) {
    reefScoringLocations.push_back(frc846::math::FieldPoint{
        reef_center + left_reef_displacement.rotate(60_deg * i, true),
        60_deg * i + 180_deg, 0_fps}
            .mirror(mirror));
    reefScoringLocations.push_back(frc846::math::FieldPoint{
        reef_center + right_reef_displacement.rotate(60_deg * i, true),
        60_deg * i + 180_deg, 0_fps}
            .mirror(mirror));
  }

  return reefScoringLocations;
}

int ReefProvider::getClosestReefSide(frc846::math::Vector2D current_pos) {
  bool mirror = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue;

  frc846::math::Vector2D reef_center = reefPoint.mirrorOnlyY(mirror).point;
  frc846::math::Vector2D reef_to_robot = current_pos - reef_center;

  units::degree_t angle = reef_to_robot.angle(true);
  while (angle < 0_deg)
    angle += 360_deg;
  while (angle > 360_deg)
    angle -= 360_deg;

  int mirror_addition = mirror ? 3 : 0;

  if (angle < 30_deg)
    return (0 + mirror_addition) % 6;
  else if (angle < 90_deg)
    return (1 + mirror_addition) % 6;
  else if (angle < 150_deg)
    return (2 + mirror_addition) % 6;
  else if (angle < 210_deg)
    return (3 + mirror_addition) % 6;
  else if (angle < 270_deg)
    return (4 + mirror_addition) % 6;
  else if (angle < 330_deg)
    return (5 + mirror_addition) % 6;

  return 0;
}