#include "commands/teleop/lock_to_reef_command.h"

#include "reef.h"

LockToReefCommand::LockToReefCommand(
    RobotContainer& container, bool is_left, frc846::math::Vector2D& base_adj)
    : frc846::robot::swerve::LockToPointCommand{&(container.drivetrain_), {},
          [&, &cnt = container, lft = is_left, &ba = base_adj](
              frc846::math::FieldPoint ctarget, frc846::math::FieldPoint start,
              bool firstLoop) {
            auto pos = cnt.drivetrain_.GetReadings().estimated_pose.position;
            int reef_target_pos = ReefProvider::getClosestReefSide(pos);

            auto ci_readings_ = cnt.control_input_.GetReadings();
            units::inch_t adj_rate =
                cnt.drivetrain_.GetPreferenceValue_unit_type<units::inch_t>(
                    "lock_adj_rate");
            if (ci_readings_.rc_n_x) {
              ba[0] -= adj_rate;
            } else if (ci_readings_.rc_p_x) {
              ba[0] += adj_rate;
            } else if (ci_readings_.rc_n_y) {
              ba[1] -= adj_rate;
            } else if (ci_readings_.rc_p_y) {
              ba[1] += adj_rate;
            }

            auto target_pos = ReefProvider::getReefScoringLocations(true, false,
                !(cnt.control_input_.GetReadings().coral_state ==
                        kCoral_ScoreL2 ||
                    cnt.control_input_.GetReadings().coral_state ==
                        kCoral_ScoreL3))[2 * reef_target_pos + (lft ? 0 : 1)];

            if (!cnt.coral_ss_.coral_end_effector.GetReadings().has_piece_) {
              piece_counter_++;
            } else {
              piece_counter_ = 0;
            }

            if (piece_counter_ >= 30 && lft) { ba[0] = -4_in; }

            auto bearing = cnt.drivetrain_.GetReadings().pose.bearing;
            target_pos.point += ba.rotate(bearing);

            return std::pair<frc846::math::FieldPoint, bool>{target_pos, true};
          }} {
  base_adj = {0_in, 0_in};
}