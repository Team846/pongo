#include "frc846/robot/swerve/path_logger.h"

#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace frc846::robot::swerve {

PathLogger::PathLogger() : Loggable("PathLogger") {
  RegisterPreference("sampling_rate_ms", 50);
}

void PathLogger::StartRecording(const std::string& filename) {
  if (is_recording_) {
    Warn("Already recording path data. Stop current recording first.");
    return;
  }

  current_filename_ = filename;
  recorded_poses_.clear();
  is_recording_ = true;
  start_time_ = frc::Timer::GetFPGATimestamp();
  Log("Started recording path data to {}.csv", current_filename_);
  Graph("recording", true);
}

bool PathLogger::StopRecording() {
  if (!is_recording_) {
    Warn("Not currently recording path data.");
    return false;
  }

  is_recording_ = false;
  Graph("recording", false);

  bool success = SavePath();

  if (success) {
    Log("Successfully saved path data to {}.csv with {} points",
        current_filename_, recorded_poses_.size());
  } else {
    Error("Failed to save path data to {}.csv", current_filename_);
  }

  return success;
}

void PathLogger::RecordPose(
    const frc846::robot::swerve::odometry::SwervePose& pose) {
  if (!is_recording_) { return; }

  PoseRecord record{.timestamp = frc::Timer::GetFPGATimestamp(), .pose = pose};

  recorded_poses_.push_back(record);
  Graph("recorded_points", static_cast<int>(recorded_poses_.size()));
}

bool PathLogger::IsRecording() const { return is_recording_; }

bool PathLogger::SavePath() {
  if (recorded_poses_.empty()) {
    Warn("No path data to save.");
    return false;
  }

  try {
    std::filesystem::create_directories(GetSavePath());

    std::string filePath = GetSavePath() + "/" + current_filename_ + ".csv";

    std::ofstream file(filePath, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
      Error("Failed to open file for writing: {}", filePath);
      return false;
    }

    file << "timestamp,x_position,y_position,bearing,velocity_x,velocity_y"
         << std::endl;

    // Write data
    for (const auto& record : recorded_poses_) {
      file << record.timestamp.to<double>() << ","
           << record.pose.position[0].to<double>() << ","
           << record.pose.position[1].to<double>() << ","
           << record.pose.bearing.to<double>() << ","
           << record.pose.velocity[0].to<double>() << ","
           << record.pose.velocity[1].to<double>() << std::endl;
    }

    file.close();
    return true;
  } catch (const std::exception& e) {
    Error("Exception while saving path data: {}", e.what());
    return false;
  }
}

std::string PathLogger::GetSavePath() const { return "/home/lvuser/paths"; }

}  // namespace frc846::robot::swerve