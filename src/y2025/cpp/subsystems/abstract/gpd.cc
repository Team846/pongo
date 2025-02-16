#include "subsystems/abstract/gpd.h"

#include <vector>

#include "frc846/math/constants.h"
#include "frc846/math/fieldpoints.h"
#include "frc846/wpilib/time.h"
#include "iostream"

GPDSubsystem::GPDSubsystem(
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::robot::GenericSubsystem<GPDReadings, GPDTarget>{"GPD"},
      drivetrain_{drivetrain} {
#ifndef _WIN32
  for (int i = 0; i < 20; i++) {
    g_field.GetObject(std::to_string(i));
  }
  frc::SmartDashboard::PutData("GPDField", &g_field);
#endif

  RegisterPreference("intake_to_cam_y", -15_in);
  RegisterPreference("intake_to_cam_x", -3_in);
  RegisterPreference("cam_h_angle", 0_deg);

  RegisterPreference("max_gp_diff", 10_in);
  RegisterPreference("use_diff_thresh", 40_in);
}

GPDTarget GPDSubsystem::ZeroTarget() const { return GPDTarget{}; }

bool GPDSubsystem::VerifyHardware() { return true; }

void GPDSubsystem::Setup() {}

frc846::math::VectorND<units::feet_per_second, 2>
GPDSubsystem::calculateVelocity(std::vector<gp_past_loc> pastPositions) {
  if (pastPositions.size() <= 2) {
    return frc846::math::VectorND<units::feet_per_second, 2>{0_fps, 0_fps};
  }

  gp_past_loc previous = pastPositions[pastPositions.size() - 1];
  gp_past_loc current = pastPositions[pastPositions.size() - 2];

  auto elapsedTime = current.time - previous.time;

  units::second_t dt = elapsedTime.count() / 1000.0 * 1_s;

  if (dt == 0_s) {
    return frc846::math::VectorND<units::feet_per_second, 2>{0_fps, 0_fps};
  }

  frc846::math::Vector2D diff = current.position - previous.position;
  units::inch_t h_diff = current.height - previous.height;
  // pastPositions.pop_back();
  units::feet_per_second_t vx = diff[0] / dt;
  units::feet_per_second_t vy = diff[1] / dt;
  units::feet_per_second_t vz = h_diff / dt;

  vx = smoothervx.Calculate(vx.to<double>()) * 1_fps;
  vy = smoothervx.Calculate(vy.to<double>()) * 1_fps;

  return frc846::math::VectorND<units::feet_per_second, 2>{vx, vy};
}

units::feet_per_second_t GPDSubsystem::calculateZVelocity(
    std::vector<gp_past_loc> pastPositions) {
  if (pastPositions.size() <= 2) { return 0_fps; }

  gp_past_loc previous = pastPositions[pastPositions.size() - 2];
  gp_past_loc current = pastPositions[pastPositions.size() - 1];

  auto elapsedTime = current.time - previous.time;
  units::second_t dt = elapsedTime.count() / 1000.0 * 1_s;

  if (dt == 0_s) { return 0_fps; }
  std::cout << std::to_string(current.height.to<double>());

  units::inch_t height_diff = current.height - previous.height;
  units::feet_per_second_t vz = height_diff / dt;

  vz = smoothervz.Calculate(vz.to<double>()) * 1_fps;
  return vz;
}

std::vector<gp_track> GPDSubsystem::update(
    std::vector<frc846::math::Vector2D>& detections,
    std::vector<units::inch_t>& heightsklk) {
  std::vector<bool> matched(detections.size(), false);
  for (size_t j = 0; j < detections.size(); j++) {
    frc846::math::Vector2D detection = detections[j];
    units::inch_t height = heightsklk[j];
    units::inch_t bestDistance = maxDistance;
    int bestIndex = -1;
    for (size_t i = 0; i < tracks.size(); i++) {
      units::inch_t dist = (tracks[i].position - detection).magnitude();
      if (dist < bestDistance && !matched[i]) {
        bestDistance = dist;
        bestIndex = static_cast<int>(i);
      }
    }

    if (bestIndex != -1) {
      tracks[bestIndex].position = detection;
      tracks[bestIndex].missedFrames = 0;
      gp_past_loc past_loc;
      past_loc.time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
      past_loc.position = detection;
      past_loc.height = height;
      tracks[bestIndex].pastPositions.push_back(past_loc);
      tracks[bestIndex].velocity =
          calculateVelocity(tracks[bestIndex].pastPositions);
      tracks[bestIndex].zvelocity =
          calculateZVelocity(tracks[bestIndex].pastPositions);
      matched[bestIndex] = true;
    } else {
      gp_track newTrack;
      newTrack.id = nextId++;
      newTrack.position = detection;
      gp_past_loc past_loc;
      past_loc.time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
      past_loc.position = detection;
      past_loc.height = height;
      newTrack.pastPositions.push_back(past_loc);
      newTrack.missedFrames = 0;
      newTrack.zvelocity = 0_fps;
      tracks.push_back(newTrack);
      matched.push_back(true);
    }

    for (size_t i = 0; i < tracks.size();) {
      if (!matched[i]) { tracks[i].missedFrames++; }
      // Remove track if missed for more than max_missed_frames
      if (tracks[i].missedFrames > max_missed_frames) {
        tracks.erase(tracks.begin() + i);
        matched.erase(matched.begin() + i);
      } else {
        i++;
      }
    }
  }

  return tracks;
}

std::pair<frc846::math::Vector2D, bool> GPDSubsystem::getBestGP(
    const std::vector<frc846::math::Vector2D> algae) {
  if (algae.size() == 0U) { return {{0_in, 0_in}, false}; }

  frc846::math::Vector2D closest_algae;

  auto robot_pose = drivetrain_->GetReadings().pose;

  if (robot_pose.velocity.magnitude() >= 2_fps) {
    units::degree_t min_angle = 180_deg;
    for (size_t i = 0; i < algae.size(); i++) {
      frc846::math::Vector2D this_algae = algae.at(i);
      units::degree_t angle = robot_pose.velocity.angleTo(this_algae, true);

      if (angle < min_angle) {
        min_angle = angle;
        closest_algae = this_algae;
      }
    }
  } else {
    units::inch_t min_dist = 1000_in;
    for (size_t i = 0; i < algae.size(); i++) {
      frc846::math::Vector2D this_algae = algae.at(i);
      units::inch_t dist = (this_algae - robot_pose.position).magnitude();

      if (dist < min_dist) {
        min_dist = dist;
        closest_algae = this_algae;
      }
    }
  }

  return {closest_algae, true};
}

GPDReadings GPDSubsystem::ReadFromHardware() {
  GPDReadings readings;
  frc846::robot::swerve::DrivetrainReadings drivetrain_readings =
      drivetrain_->GetReadings();

  g_field.SetRobotPose(frc846::math::FieldPoint::field_size_y -
                           drivetrain_readings.pose.position[1],
      drivetrain_readings.pose.position[0],
      180_deg - drivetrain_readings.pose.bearing);

  std::vector<double> distances = gpdTable->GetNumberArray("distances", {});

  auto latency_entry = gpdTable->GetEntry("tl");
  units::second_t nt_delay =
      frc846::wpilib::CurrentFPGATime() -
      units::microsecond_t(latency_entry.GetLastChange());
  Graph("nt_delay", nt_delay);
  units::second_t latency = latency_entry.GetDouble(0.005) * 1_s + nt_delay;
  Graph("latency", latency);

  std::vector<double> theta_x = gpdTable->GetNumberArray("tx", {});
  std::vector<double> h = gpdTable->GetNumberArray("heights", {});

  readings.gamepieces.clear();

  for (size_t i = 0; i < distances.size() && i < theta_x.size() && i < h.size();
       ++i) {
    readings.gamepieces.push_back(
        frc846::math::Vector2D{units::inch_t(distances[i]),
            drivetrain_readings.pose.bearing -
                drivetrain_readings.yaw_rate * latency +
                units::degree_t(theta_x[i]) +
                GetPreferenceValue_unit_type<units::degree_t>("cam_h_angle"),
            true} +
        drivetrain_readings.pose.position -
        frc846::math::Vector2D{drivetrain_readings.pose.velocity[0] * latency,
            drivetrain_readings.pose.velocity[1] * latency} +
        frc846::math::Vector2D{
            GetPreferenceValue_unit_type<units::inch_t>("intake_to_cam_x"),
            GetPreferenceValue_unit_type<units::inch_t>("intake_to_cam_y")}
            .rotate(drivetrain_readings.pose.bearing));
    readings.heights.push_back(units::inch_t(h[i]));
  }

  int num_gps = readings.gamepieces.size();

  Graph("num_gps", num_gps);

  gp_spin_ += 5_deg;

  Graph("next_id", nextId);

  readings.tracks = update(readings.gamepieces, readings.heights);

#ifndef _WIN32
  for (int i = 0; i < std::min(20, (int)tracks.size()); i++) {
    gp_track gp = tracks[i];
    Graph("algaex" + std::to_string(gp.id % 5), gp.position[0]);
    Graph("algaey" + std::to_string(gp.id % 5), gp.position[1]);
    Graph("algaevy" + std::to_string(gp.id % 5), gp.velocity[1]);
    Graph("algaevx" + std::to_string(gp.id % 5), gp.velocity[0]);
    Graph("algaevz" + std::to_string(gp.id % 5), gp.zvelocity);
    g_field.GetObject(std::to_string(i))
        ->SetPose(frc846::math::FieldPoint::field_size_y - gp.position[1],
            gp.position[0], gp_spin_);
  }
  for (int i = std::min(20, num_gps); i < 20; i++) {
    g_field.GetObject(std::to_string(i))->SetPose(100_m, 100_m, 0_deg);
  }
#endif
  return readings;
}

void GPDSubsystem::WriteToHardware(GPDTarget target) {}