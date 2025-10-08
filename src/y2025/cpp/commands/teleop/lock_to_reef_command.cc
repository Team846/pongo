#include "commands/teleop/lock_to_reef_command.h"

#include "frc846/robot/swerve/wait_until_close.h"
#include "reef.h"

int LockToReefCommand::piece_counter_ = 0;

LockToReefCommand::LockToReefCommand(
    RobotContainer& container, bool is_left, frc846::math::Vector2D& base_adj)
    : frc846::robot::swerve::LockToPointCommand{&(container.drivetrain_), {},
          [&, &cnt = container, lft = is_left, &ba = base_adj,
              pc = piece_counter_](frc846::math::FieldPoint ctarget,
              frc846::math::FieldPoint start, bool firstLoop) {
            auto pos = cnt.drivetrain_.GetReadings().estimated_pose.position;
            int reef_target_pos = ReefProvider::getClosestReefSide(pos);

            auto ci_readings_ = cnt.control_input_.GetReadings();
            // units::inch_t adj_rate =
            //     cnt.drivetrain_.GetPreferenceValue_unit_type<units::inch_t>(
            //         "lock_adj_rate");
            // if (ci_readings_.rc_n_x) {
            //   ba[0] -= adj_rate;
            // } else if (ci_readings_.rc_p_x) {
            //   ba[0] += adj_rate;
            // } else if (ci_readings_.rc_n_y) {
            //   ba[1] -= adj_rate;
            // } else if (ci_readings_.rc_p_y) {
            //   ba[1] += adj_rate;
            // }

            auto target_pos = ReefProvider::getReefScoringLocations(true, false,
                !(cnt.control_input_.GetReadings().coral_state ==
                        kCoral_ScoreL2 ||
                    cnt.control_input_.GetReadings().coral_state ==
                        kCoral_ScoreL3))[2 * reef_target_pos + (lft ? 0 : 1)];

            if (!cnt.coral_ss_.coral_end_effector.GetReadings().has_piece_) {
              (LockToReefCommand::piece_counter_)++;
            } else {
              (LockToReefCommand::piece_counter_) = 0;
            }

            if (ci_readings_.level_one) {
              ba[0] = -15_in;
              if (lft) {
                ba[0] -= 0.5_in;
              } else {
                ba[0] += 3.5_in;
              }
              ba[1] = -9.0_in;
            } else if (lft && (LockToReefCommand::piece_counter_) > 30) {
              ba[0] = -6_in;
            }

            auto bearing = cnt.drivetrain_.GetReadings().pose.bearing;
            target_pos.point += ba.rotate(bearing);

            return std::pair<frc846::math::FieldPoint, bool>{target_pos, true};
          }} {
  base_adj = {0_in, 0_in};
}