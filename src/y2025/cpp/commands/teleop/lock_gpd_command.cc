#include "commands/teleop/lock_gpd_command.h"

#include <iostream>

LockGPDCommand::LockGPDCommand(RobotContainer& container)
    : frc846::robot::swerve::LockToPointCommand{&(container.drivetrain_), {},
          [&, &cnt = container](frc846::math::FieldPoint ctarget,
              frc846::math::FieldPoint start, bool firstLoop) {
            auto gpd_readings = cnt.GPD_.GetReadings();
            const auto [gpd_pos, valid] =
                cnt.GPD_.getBestGP(gpd_readings.gamepieces);
            // Graph("valid_gamepiece", valid);

            if (!valid)
              return std::pair<frc846::math::FieldPoint, bool>{{}, false};

            auto diff_vec = gpd_pos - ctarget.point;
            auto dist_prev =
                (ctarget.point - cnt.drivetrain_.GetReadings().pose.position)
                    .magnitude();
            // Graph("diff_vec_magnitude_gpd", diff_vec.magnitude());
            if ((!firstLoop) &&
                (diff_vec.magnitude() >
                    cnt.GPD_.GetPreferenceValue_unit_type<units::inch_t>(
                        "max_gp_diff")) &&
                dist_prev <
                    cnt.GPD_.GetPreferenceValue_unit_type<units::inch_t>(
                        "use_diff_thresh"))
              return std::pair<frc846::math::FieldPoint, bool>{{}, false};

            auto tt_vec =
                (gpd_pos - cnt.drivetrain_.GetReadings().pose.position);
            if (tt_vec.magnitude() > 6_in)
              return std::pair<frc846::math::FieldPoint, bool>{
                  frc846::math::FieldPoint{gpd_pos,
                      (gpd_pos - cnt.drivetrain_.GetReadings().pose.position)
                          .angle(true),
                      0_fps},
                  true};
            else
              return std::pair<frc846::math::FieldPoint, bool>{{}, false};
          }} {}