#include "commands/teleop/lock_to_net_command.h"

LockToNetCommand::LockToNetCommand(RobotContainer& container,
    frc846::math::Vector2D& base_adj)
    : frc846::robot::swerve::LockToPointCommand{&(container.drivetrain_), {},
          [&, &cnt = container, &ba = base_adj](
              frc846::math::FieldPoint ctarget, frc846::math::FieldPoint start,
              bool firstLoop) {
            frc846::math::Vector2D pos =
                cnt.drivetrain_.GetReadings().estimated_pose.position;
            units::degree_t bearing =
                cnt.drivetrain_.GetReadings().estimated_pose.bearing;

            auto ci_readings_ = cnt.control_input_.GetReadings();
            units::inch_t adj_rate =
                cnt.drivetrain_.GetPreferenceValue_unit_type<units::inch_t>(
                    "lock_net_adj_rate");
            if (ci_readings_.rc_n_x) {
              ba[0] -= adj_rate;
            } else if (ci_readings_.rc_p_x) {
              ba[0] += adj_rate;
            } else if (ci_readings_.rc_n_y) {
              ba[1] -= adj_rate;
            } else if (ci_readings_.rc_p_y) {
              ba[1] += adj_rate;
            }

            units::inch_t net_offest = 45_in;
            frc846::math::Vector2D net_pos = frc846::math::Vector2D{pos[0],
                frc846::math::FieldPoint::field_size_y / 2 - net_offest};
            frc846::math::FieldPoint target_pos = frc846::math::FieldPoint{
                net_pos + ba.rotate(bearing), 0_deg, 0_fps};

            if (bearing > 150_deg) { target_pos = target_pos.mirror(true); }

            return std::pair<frc846::math::FieldPoint, bool>{target_pos, true};
          }} {
  base_adj = {0_in, 0_in};
}
