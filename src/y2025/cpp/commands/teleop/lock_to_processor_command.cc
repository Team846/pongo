#include "commands/teleop/lock_to_processor_command.h"

#include "frc/DriverStation.h"
#include "reef.h"

LockToProcessorCommand::LockToProcessorCommand(
    RobotContainer& container, frc846::math::Vector2D& base_adj)
    : frc846::robot::swerve::LockToPointCommand{&(container.drivetrain_), {},
          [&, &cnt = container, &ba = base_adj](
              frc846::math::FieldPoint ctarget, frc846::math::FieldPoint start,
              bool firstLoop) {
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

            frc846::math::FieldPoint target_pos = {
                {295.75_in, 235.73_in}, -90_deg, 0_fps};
            target_pos = target_pos.mirror(
                frc::DriverStation::GetAlliance() ==
                frc::DriverStation::kBlue);  // TODO: make these points
                                             // scriptable

            auto bearing = cnt.drivetrain_.GetReadings().pose.bearing;
            target_pos.point += ba;

            return std::pair<frc846::math::FieldPoint, bool>{target_pos, true};
          }} {
  base_adj = {0_in, 0_in};
}