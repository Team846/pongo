#include "frc846/robot/calculators/AprilTagCalculator.h"

#include <iostream>
#include <units/math.h>

#include "frc846/math/collection.h"
#include "frc846/robot/GenericRobot.h"
#include "frc846/wpilib/time.h"

namespace frc846::robot::calculators {

ATCalculatorOutput AprilTagCalculator::calculate(ATCalculatorInput input) {
  ATCalculatorOutput output;

  double totalTagWeight = 0;
  double variance = 0;

  std::vector<frc846::math::Line> sight_lines[constants_.cams];
  int tagsSeen = 0;

  for (int i = 0; i < constants_.cams; i++) {
    units::second_t delay =
        frc846::wpilib::CurrentFPGATime() -
        units::microsecond_t(
            constants_.april_tables.at(i)->GetEntry("tl").GetLastChange());
    units::second_t tl =
        units::second_t(constants_.april_tables.at(i)->GetNumber("tl", -1)) +
        delay;
    if (delay > 3.5 * frc846::robot::GenericRobot::kPeriod) { continue; }

    std::vector<double> tx_nums =
        constants_.april_tables.at(i)->GetNumberArray("tx", {});
    std::vector<double> distances_num =
        constants_.april_tables.at(i)->GetNumberArray("distances", {});
    std::vector<units::degree_t> tx;
    std::vector<units::inch_t> distances;
    for (double tx_num : tx_nums) {
      tx.push_back(units::degree_t(tx_num));
    };
    for (double distance_num : distances_num) {
      distances.push_back(units::inch_t(distance_num));
    };

    std::vector<double> tags =
        constants_.april_tables.at(i)->GetNumberArray("tags", {});
    units::degree_t bearingAtCapture =
        input.pose.bearing -
        input.angular_velocity *
            (tl + input.bearing_latency);  // TODO: modify latency compensation
                                           // if there is angular acceleration?

    // CASE 1: Triangulate with 2 tags
    // if (tags.size() >= 2 && tags.size() == distances.size() &&
    //     tags.size() == tx.size()) {
    //   frc846::math::Vector2D loc_tag_1 = {
    //       constants_.tag_locations[tags[0]].x_pos,
    //       constants_.tag_locations[tags[0]].y_pos};
    //   frc846::math::Vector2D loc_tag_2 = {
    //       constants_.tag_locations[tags[1]].x_pos,
    //       constants_.tag_locations[tags[1]].y_pos};

    //   auto rotated_tag1 = loc_tag_1.rotate(bearingAtCapture, false);
    //   auto rotated_tag2 = loc_tag_2.rotate(bearingAtCapture, false);

    //   double tag1Slope = 1 / units::math::tan(tx[0]);
    //   double tag2Slope = 1 / units::math::tan(tx[1]);

    //   units::foot_t camera_x = ((loc_tag_2[1] - tag2Slope * loc_tag_2[0]) -
    //                                (loc_tag_1[1] - tag1Slope * loc_tag_1[0]))
    //                                /
    //                            (tag1Slope - tag2Slope);
    //   units::foot_t camera_y =
    //       (camera_x - loc_tag_1[0]) * tag1Slope + loc_tag_1[1];

    //   frc846::math::Vector2D camera_pos = {camera_x, camera_y};
    //   frc846::math::Vector2D cam_offset = {
    //       constants_.camera_x_offsets[i], constants_.camera_y_offsets[i]};

    //   frc846::math::Vector2D uncompensatedPos = camera_pos - cam_offset;
    //   uncompensatedPos = uncompensatedPos.rotate(bearingAtCapture);

    //   frc846::math::Vector2D velComp = {
    //       (input.pose.velocity[0] + input.old_pose.velocity[0]) / 2 *
    //           (tl + input.fudge_latency[i]),
    //       (input.pose.velocity[1] + input.old_pose.velocity[1]) / 2 *
    //           (tl + input.fudge_latency[i])};
    //   //   if (frc846::math::modulo(fieldTag1Tx, 90_deg)>15_deg
    //   //   &&frc846::math::modulo(fieldTag2Tx, 90_deg)>15_deg){
    //   //
    //   frc846::math::Vector2D est_pos = uncompensatedPos + velComp;

    //   //   output.pos += (est_pos) * 1.2;
    //   //   variance +=
    //   //       1 / std::max(
    //   //               (input.triangularVarianceCoeff *
    //   // std::sqrt((distances.at(0)+distances.at(1)).to<double>()/2)
    //   //                   * std::pow(
    //   //                       1 +
    //   // input.pose.velocity.magnitude().to<double>()/12,
    //   //                       2) *
    //   //                   std::pow(1 +
    //   input.angular_velocity.to<double>()/300,
    //   //                   2)),
    //   //               0.0000000001);
    //   //   totalTagWeight +=
    //   //   180/(distances.at(0).to<double>()+distances.at(1).to<double>());

    // }
    // // CASE 2: Single tag estimate
    // else if (tags.size() == 1 && distances.size() == 1 && tx.size() == 1) {
    //   if (constants_.tag_locations.contains(tags[0])) {
    //     frc846::math::Vector2D velComp = {
    //         (input.pose.velocity[0] + input.old_pose.velocity[0]) / 2 *
    //             (tl + input.fudge_latency[i]),
    //         (input.pose.velocity[1] + input.old_pose.velocity[1]) / 2 *
    //             (tl + input.fudge_latency[i])};
    //     if (distances.at(0) < 140_in) {
    //       output.pos += (getPos(bearingAtCapture, tx.at(0), distances.at(0),
    //                          tags.at(0), i) +
    //                         velComp) *
    //                     (48) / distances.at(0).to<double>();
    //       variance +=  // -0.247, 0.0699
    //           1 /
    //           std::max(
    //               (input.aprilVarianceCoeff * distances.at(0).to<double>() *
    //                   std::pow(
    //                       1 + input.pose.velocity.magnitude().to<double>() /
    //                       12, 2) *
    //                   std::pow(
    //                       1 + input.angular_velocity.to<double>() / 300, 2)),
    //               0.0000000001);
    //       totalTagWeight += (48) / distances.at(0).to<double>();
    //     }
    //   }
    // }
    if (!(tags.size()==distances.size() && tags.size()==tx.size())){
        continue;
    }
    for (int j = 0; j < tags.size(); j++) {
      if (constants_.tag_locations.contains(tags.at(j)) &&
          distances.at(j) < 140_in) {
        frc846::math::Vector2D cam_to_tag{
            distances.at(j), tx.at(j) + bearingAtCapture, true};
        frc846::math::Vector2D tag_pos{
            constants_.tag_locations[tags.at(j)].x_pos,
            constants_.tag_locations[tags.at(j)].y_pos};
        frc846::math::Vector2D uncomp_cam_pos = tag_pos - cam_to_tag;

        frc846::math::Vector2D velComp = {
            (input.pose.velocity[0] + input.old_pose.velocity[0]) / 2 *
                (tl + input.fudge_latency[i]),
            (input.pose.velocity[1] + input.old_pose.velocity[1]) / 2 *
                (tl + input.fudge_latency[i])};

        frc846::math::Vector2D comp_cam_pos = uncomp_cam_pos + velComp;

        units::degree_t comp_angle = (tag_pos - comp_cam_pos).angle();
        frc846::math::Line sight_line{tag_pos, comp_angle};

        frc846::math::VectorND<units::inch, 2> center_to_cam{
            constants_.camera_x_offsets[i],
            constants_.camera_y_offsets[i],
        };

        sight_line.translate(center_to_cam.rotate(bearingAtCapture) * -1);
        sight_lines[i].push_back(sight_line);
        if (distances.at(j)<30_in){
            output.pos=(getPos(bearingAtCapture, tx.at(j), distances.at(j),
                           tags.at(j), i) +
                          velComp);
            output.variance = std::max(
                (input.aprilVarianceCoeff * std::pow(distances.at(j).to<double>(), 3) *
                    std::pow(
                        1 + input.pose.velocity.magnitude().to<double>() / 12,
                        2) *
                    std::pow(1 + input.angular_velocity.to<double>() / 300, 2)),
                0.0000000001);

    correction = output.pos - input.pose.position;
    return output;
        }
        output.pos += (getPos(bearingAtCapture, tx.at(j), distances.at(j),
                           tags.at(j), i) +
                          velComp) *
                      (48) / distances.at(j).to<double>();
                      
        variance +=
            1 /
            std::max(
                (input.aprilVarianceCoeff * std::pow(distances.at(j).to<double>(), 3) *
                    std::pow(
                        1 + input.pose.velocity.magnitude().to<double>() / 12,
                        2) *
                    std::pow(1 + input.angular_velocity.to<double>() / 300, 2)),
                0.0000000001);
        totalTagWeight += (48) / distances.at(j).to<double>();

        tagsSeen++;
      }
    }
  }

  if (tagsSeen == 1) {
    output.variance = 1 / variance;
    output.pos = output.pos / totalTagWeight;
    correction = output.pos - input.pose.position;
    return output;
  }


  // Double Cam Triangulation
  frc846::math::Line first_line = {{-1_in, -1_in}, 0_deg};
  for (int i = 0; i < constants_.cams; i++) {
    if (sight_lines[i].size() > 0) {
      if (first_line.point[0] != -1_in) {
        // Double Cam Triangulation!
        output.pos = first_line.intersect(sight_lines[i].at(0));
        output.variance =
            std::max(
                (input.triangularVarianceCoeff *
                    std::pow(
                        1 + input.pose.velocity.magnitude().to<double>() / 12,
                        2) *
                    std::pow(1 + input.angular_velocity.to<double>() / 300, 2)),
                0.0000000001);  // TODO: make better

        correction = output.pos - input.pose.position;
        return output;
      } else {
        first_line = sight_lines[i].at(0);
      }
    }
  }


  // Single Cam?
  for (int i = 0; i < constants_.cams; i++) {
    if (sight_lines[i].size() > 1) {
      // Double Tag, Single Cam Triangulation!
      output.pos = sight_lines[i].at(1).intersect(sight_lines[i].at(0));
      output.variance =
          std::max(
              (input.triangularVarianceCoeff *
                  std::pow(
                      1 + input.pose.velocity.magnitude().to<double>() / 12,
                      2) *
                  std::pow(1 + input.angular_velocity.to<double>() / 300, 2)),
              0.0000000001);  // TODO: make better

      correction = output.pos - input.pose.position;
      return output;
    }
  }
  return {input.pose.position + correction, -1};
}

frc846::math::VectorND<units::inch, 2> AprilTagCalculator::getPos(
    units::degree_t bearing, units::degree_t theta, units::inch_t distance,
    int tag, int camera) {
  frc846::math::VectorND<units::inch, 2> cam_to_tag{
      distance * units::math::sin(theta),
      distance * units::math::cos(theta),
  };
  frc846::math::VectorND<units::inch, 2> center_to_cam{
      constants_.camera_x_offsets[camera],
      constants_.camera_y_offsets[camera],
  };

  frc846::math::VectorND<units::inch, 2> local_tag_pos =
      center_to_cam + cam_to_tag;
  local_tag_pos = local_tag_pos.rotate(bearing);

  return {
      constants_.tag_locations[tag].x_pos - local_tag_pos[0],
      constants_.tag_locations[tag].y_pos - local_tag_pos[1],
  };
}
}