#pragma once

#include "frc846/base/Loggable.h"
#include "frc846/math/fieldpoints.h"

class ReefProvider {
public:
  /*
  getReefScoringLocations()

  Returns a vector of FieldPoints representing the locations of the reef scoring
  locations on the field. Order: LRLR(etc.). Starts furthest from alliance wall,
  then moves CW.
  */
  static std::vector<frc846::math::FieldPoint> getReefScoringLocations(
      bool mirrorIfBlue = true, bool prePoint = false, bool l4 = true);

  /*
  getClosestReefSide()

  Returns the side (0, 1, 2, 3, 4, 5) of the closest reef scoring location to
  the robot. Starts furthest from alliance wall, then moves CW.
  */
  static int getClosestReefSide(frc846::math::Vector2D current_pos);

  /*
  getReefNumAuto()

  Returns the reef number (0-11) of the reef scoring location after mirroring
  */
  static int getReefNumAuto(int number_on_right, bool left_side);

  static frc846::math::FieldPoint reefPoint;
};