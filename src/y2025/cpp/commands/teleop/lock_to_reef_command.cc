#include "commands/teleop/lock_to_reef_command.h"

#include "reef.h"

LockToReefCommand::LockToReefCommand(RobotContainer& container, bool is_left)
    : frc846::robot::swerve::LockToPointCommand{&(container.drivetrain_),
          ReefProvider::getReefScoringLocations()[is_left ? 0 : 1],
          [&, &cnt = container, &ba = base_adj, &lft = is_left](
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

            auto target_pos =
                ReefProvider::getReefScoringLocations()[2 * reef_target_pos +
                                                        (lft ? 0 : 1)];

            target_pos.point +=
                ba.rotate(cnt.drivetrain_.GetReadings().pose.bearing);

            return std::pair<frc846::math::FieldPoint, bool>{target_pos, true};
          }} {}