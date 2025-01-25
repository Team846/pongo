#include "reef.h"

#include <iostream>

std::vector<frc846::math::FieldPoint> ReefProvider::getReefScoringLocations() {
  std::vector<frc846::math::FieldPoint> reefScoringLocations;

  // TODO: add real values
  // TODO: add reef center
  frc846::math::Vector2D left_reef_displacement =
      frc846::math::Vector2D{8.0_in, 40.0_in};

  frc846::math::Vector2D right_reef_displacement =
      frc846::math::Vector2D{-8.0_in, 40.0_in};

  for (int i = 0; i < 6; i++) {
    reefScoringLocations.push_back(frc846::math::FieldPoint{
        left_reef_displacement.rotate(60_deg * i, true), 60_deg * i + 180_deg,
        0_fps});
    reefScoringLocations.push_back(frc846::math::FieldPoint{
        right_reef_displacement.rotate(60_deg * i, true), 60_deg * i + 180_deg,
        0_fps});
  }

  return reefScoringLocations;
}

int ReefProvider::getClosestReefSide(frc846::math::Vector2D current_pos) {
  // TODO: include center reef, perhaps hysteresis

  units::degree_t angle = current_pos.angle(true);
  while (angle < 0_deg)
    angle += 360_deg;
  while (angle > 360_deg)
    angle -= 360_deg;

  if (angle < 30_deg)
    return 0;
  else if (angle < 90_deg)
    return 1;
  else if (angle < 150_deg)
    return 2;
  else if (angle < 210_deg)
    return 3;
  else if (angle < 270_deg)
    return 4;
  else if (angle < 330_deg)
    return 5;

  return 0;
}