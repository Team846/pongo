#pragma once

#include <frc/Timer.h>
#include <units/time.h>

#include <fstream>
#include <string>
#include <vector>

#include "frc846/base/Loggable.h"
#include "frc846/robot/swerve/odometry/swerve_pose.h"

namespace frc846::robot::swerve {

/**
 * PathLogger
 *
 * A class to record the robot's path using odometry data and save it to a file.
 */
class PathLogger : public frc846::base::Loggable {
public:
  PathLogger();

  /**
   * Starts recording the robot's path.
   *
   * @param filename The filename to save the path to (without extension).
   */
  void StartRecording(const std::string& filename);

  /**
   * Stops recording the robot's path and saves it to a file.
   *
   * @return True if the path was successfully saved, false otherwise.
   */
  bool StopRecording();

  /**
   * Adds a pose to the recording.
   *
   * @param pose The current pose of the robot.
   */
  void RecordPose(const frc846::robot::swerve::odometry::SwervePose& pose);

  /**
   * Checks if the logger is currently recording.
   *
   * @return True if recording, false otherwise.
   */
  bool IsRecording() const;

private:
  struct PoseRecord {
    units::second_t timestamp;
    frc846::robot::swerve::odometry::SwervePose pose;
  };

  std::vector<PoseRecord> recorded_poses_;
  bool is_recording_ = false;
  std::string current_filename_;
  units::second_t start_time_;

  /**
   * Saves the recorded path to a file.
   *
   * @return True if the path was successfully saved, false otherwise.
   */
  bool SavePath();

  /**
   * Gets the path where files should be saved on the RoboRIO.
   *
   * @return The path to save files to.
   */
  std::string GetSavePath() const;
};

}  // namespace frc846::robot::swerve