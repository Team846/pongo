#include "autos/auton_seqs.h"

#include <frc2/command/WaitCommand.h>

#include "frc846/robot/swerve/aim_command.h"
#include "frc846/robot/swerve/drive_to_point_command.h"

using INSTANT = frc2::InstantCommand;
using SEQUENCE = frc2::SequentialCommandGroup;
using WAIT = frc2::WaitCommand;

using FPT = frc846::math::FieldPoint;

#define MAX_ACCEL_3PC 15_fps_sq
#define MAX_DECEL_3PC 15_fps_sq
#define MAX_VEL_3PC 12_fps

#define MAX_ACCEL_1PC 7_fps_sq
#define MAX_DECEL_1PC 7_fps_sq
#define MAX_VEL_1PC 6_fps

#define MAX_ACCEL_LEAVE 10_fps_sq
#define MAX_DECEL_LEAVE 10_fps_sq
#define MAX_VEL_LEAVE 10_fps

#define AUTO_NAME(default_name)                                \
  std::string(default_name) + (is_blue_side ? "_B_" : "_R_") + \
      (is_left_side ? "L" : "R")

#define MKPT(x, y, bearing, velocity) \
  FPT{{x, y}, bearing, velocity}.mirror(is_blue_side).mirrorOnlyX(!is_left_side)

#define START(x, y, start_bearing)                               \
  INSTANT {                                                      \
    [&, blue = is_blue_side, left = is_left_side]() {            \
      FPT start_point{{x, y}, start_bearing, 0_fps};             \
      start_point = start_point.mirror(blue).mirrorOnlyX(!left); \
      container.drivetrain_.SetPosition(start_point.point);      \
      container.drivetrain_.SetBearing(start_point.bearing);     \
    }                                                            \
  }

#define DRIVE(auto_name, x, y, bearing, final_velocity)                   \
  frc846::robot::swerve::DriveToPointCommand {                            \
    &(container.drivetrain_), MKPT(x, y, bearing, final_velocity),        \
        MAX_VEL_##auto_name, MAX_ACCEL_##auto_name, MAX_DECEL_##auto_name \
  }

#define AIM(bearing)                                                     \
  frc846::robot::swerve::AimCommand {                                    \
    &(container.drivetrain_), bearing + (is_blue_side ? 180_deg : 0_deg) \
  }

ThreePieceAuto::ThreePieceAuto(
    RobotContainer& container, bool is_blue_side, bool is_left_side)
    : frc846::robot::GenericCommandGroup<RobotContainer, ThreePieceAuto,
          SEQUENCE>{container, AUTO_NAME("3PC"),
          SEQUENCE{
              START(3_ft, 4_ft, 0_deg),
              WAIT{0.25_s},
              DRIVE(3PC, 0_ft, 7_ft, -30_deg, 0_fps),
              WAIT{1_s},
              DRIVE(3PC, 2_ft, 8_ft, -100_deg, 15_fps),
              DRIVE(3PC, 6_ft, 16_ft, -126_deg, 0_fps),
              DRIVE(3PC, 1_ft, 12_ft, -120_deg, 0_fps),
              WAIT{1_s},
              DRIVE(3PC, 6_ft, 16_ft, -126_deg, 0_fps),
              DRIVE(3PC, 1.5_ft, 11.5_ft, -120_deg, 0_fps),
          }} {}

OnePieceAndNetAuto::OnePieceAndNetAuto(
    RobotContainer& container, bool is_blue_side, bool is_left_side)
    : frc846::robot::GenericCommandGroup<RobotContainer, OnePieceAndNetAuto,
          SEQUENCE>{container, AUTO_NAME("1PCN"),
          SEQUENCE{
              START(0_ft, 4_ft, 0_deg),
              WAIT{0.25_s},
              DRIVE(1PC, 0_ft, 7_ft, 0_deg, 0_fps),
              WAIT{1_s},
              DRIVE(1PC, 4_ft, 4_ft, 180_deg, 0_fps),
              DRIVE(1PC, 4_ft, 3_ft, 180_deg, 0_fps),
          }} {}

LeaveAuto::LeaveAuto(
    RobotContainer& container, bool is_blue_side, bool is_left_side)
    : frc846::robot::GenericCommandGroup<RobotContainer, LeaveAuto, SEQUENCE>{
          container, AUTO_NAME("LEAVE"),
          SEQUENCE{
              START(3_ft, 4_ft, 0_deg),
              WAIT{0.25_s},
              DRIVE(LEAVE, 3_ft, 7_ft, 0_deg, 0_fps),
          }} {}