// #include "autos/GenericAuto.h"

// #include <frc2/command/WaitCommand.h>

// #include "autos/ActionMaker.h"
// #include "frc846/robot/swerve/drive_to_point_command.h"

// GenericAuto::GenericAuto(
//     RobotContainer& container, AutoData data, bool is_blue_side)
//     : frc846::robot::GenericCommandGroup<RobotContainer, GenericAuto,
//           frc2::SequentialCommandGroup>{container, data.name,
//           frc2::SequentialCommandGroup{
//               std::move(buildActionsGroup(data, container, is_blue_side))}}
//               {}

// std::vector<std::unique_ptr<frc2::Command>> GenericAuto::buildActionsGroup(
//     AutoData data, RobotContainer& container, bool is_blue_side) {
//   std::vector<std::unique_ptr<frc2::Command>> cmds{};
//   cmds.push_back(std::make_unique<frc2::InstantCommand>(
//       [&, auto_data = data, is_blue = is_blue_side] {
//         Log("Starting Auto: {}.", auto_data.name);

//         int mirror = is_blue ? (int)auto_data.blue : (int)auto_data.red;

//         auto start = auto_data.start;

//         Log("Pre-flip: Setting start to x{} y{}.",
//         start.point[0].to<double>(),
//             start.point[1].to<double>());
//         Log("Pre-flip: Setting bearing to {} deg.",
//         start.bearing.to<double>());

//         if (mirror == 2) {
//           start = auto_data.start.mirrorOnlyY(true);
//           Log("Flipping Only Y.");
//         } else if (mirror == 1) {
//           start = auto_data.start.mirror(true);
//           Log("Flipping.");
//         }

//         Log("Setting mirror to {} for blue side {}", mirror, is_blue);
//         Log("Default mirror red {} blue {}", (int)auto_data.red,
//             (int)auto_data.blue);
//         Log("Setting start to x{} y{}.", start.point[0].to<double>(),
//             start.point[1].to<double>());
//         Log("Setting bearing to {} deg.", start.bearing.to<double>());

//         container.drivetrain_.SetPosition(start.point);
//         container.drivetrain_.SetBearing(start.bearing);
//       }));
//   for (auto& action : data.actions) {
//     if (auto* action_name = std::get_if<std::string>(&action)) {
//       cmds.push_back(ActionMaker::GetAction(*action_name, container));
//     } else if (auto* fp = std::get_if<std::vector<frc846::math::FieldPoint>>(
//                    &action)) {
//       cmds.push_back(
//           std::make_unique<frc846::robot::swerve::DriveToPointCommand>(
//               &(container_.drivetrain_), *fp,
//               is_blue_side ? (int)data.blue : (int)data.red));
//     }
//   }

//   return cmds;
// }