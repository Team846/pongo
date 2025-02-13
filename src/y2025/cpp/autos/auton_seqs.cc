#include "autos/auton_seqs.h"

#include <frc2/command/WaitCommand.h>

#include "frc846/robot/swerve/aim_command.h"
#include "frc846/robot/swerve/drive_to_point_command.h"
#include "reef.h"

using INSTANT = frc2::InstantCommand;
using SEQUENCE = frc2::SequentialCommandGroup;
using WAIT = frc2::WaitCommand;

using FPT = frc846::math::FieldPoint;

#define MAX_ACCEL_3PC 20_fps_sq
#define MAX_DECEL_3PC 20_fps_sq
#define MAX_VEL_3PC 12_fps

#define MAX_ACCEL_1PC 7_fps_sq
#define MAX_DECEL_1PC 7_fps_sq
#define MAX_VEL_1PC 6_fps

#define MAX_ACCEL_LEAVE 10_fps_sq
#define MAX_DECEL_LEAVE 10_fps_sq
#define MAX_VEL_LEAVE 10_fps

#define START_Y (311.5_in - 15_in)

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

#define DRIVE_TO_SOURCE(auto_name)                                        \
  frc846::robot::swerve::DriveToPointCommand {                            \
    &(container.drivetrain_), MKPT(50_in, 68.5_in, 36_deg, 0_fps),        \
        MAX_VEL_##auto_name, MAX_ACCEL_##auto_name, MAX_DECEL_##auto_name \
  }

#define DRIVE_TO_REEF(auto_name, number_on_right)                         \
  frc846::robot::swerve::DriveToPointCommand {                            \
    &(container.drivetrain_),                                             \
        ReefProvider::getReefScoringLocations(false)[number_on_right]     \
            .mirror(is_blue_side)                                         \
            .mirrorOnlyX(is_left_side)                                    \
            .flipDirection(),                                             \
        MAX_VEL_##auto_name, MAX_ACCEL_##auto_name, MAX_DECEL_##auto_name \
  }

#define AIM(bearing)                                                     \
  frc846::robot::swerve::AimCommand {                                    \
    &(container.drivetrain_), bearing + (is_blue_side ? 180_deg : 0_deg) \
  }

#include "commands/general/algal_position_command.h"
#include "commands/general/coral_position_command.h"

#define ALGAL_POS(where, score) \
  AlgalPositionCommand { container, where, score }
#define CORAL_POS(where, score) \
  CoralPositionCommand { container, where, score }

#include <frc2/command/ParallelDeadlineGroup.h>

#define PARALLEL_DEADLINE(deadline, parallel) \
  frc2::ParallelDeadlineGroup { deadline, parallel }

ThreePieceAuto::ThreePieceAuto(
    RobotContainer& container, bool is_blue_side, bool is_left_side)
    : frc846::robot::GenericCommandGroup<RobotContainer, ThreePieceAuto,
          SEQUENCE>{container, AUTO_NAME("3PC"),
          SEQUENCE{
              START(158.5_in - 73.25_in, START_Y, 180_deg),
              WAIT{0.25_s},
              DRIVE_TO_REEF(3PC, 3),
              CORAL_POS(kCoral_ScoreL4, true),
              WAIT{1_s},
              CORAL_POS(kCoral_StowNoPiece, false),
              DRIVE(3PC, 75_in, 180_in, 40_deg, 10_fps),
              DRIVE_TO_SOURCE(3PC),
              PARALLEL_DEADLINE(DRIVE_TO_REEF(3PC, 4),
                  CORAL_POS(kCoral_StowWithPiece, false)),
              CORAL_POS(kCoral_ScoreL4, true),
              WAIT{1_s},
              CORAL_POS(kCoral_StowNoPiece, false),
              DRIVE_TO_SOURCE(3PC),
              PARALLEL_DEADLINE(DRIVE_TO_REEF(3PC, 5),
                  CORAL_POS(kCoral_StowWithPiece, false)),
              CORAL_POS(kCoral_ScoreL4, true),
              WAIT{1_s},
          }} {}

OnePieceAndNetAuto::OnePieceAndNetAuto(
    RobotContainer& container, bool is_blue_side, bool is_left_side)
    : frc846::robot::GenericCommandGroup<RobotContainer, OnePieceAndNetAuto,
          SEQUENCE>{container, AUTO_NAME("1PCN"),
          SEQUENCE{
              START(158.5_in, START_Y, 180_deg),
              WAIT{0.25_s},
              DRIVE_TO_REEF(1PC, 1),
              CORAL_POS(kCoral_ScoreL4, true),
              WAIT{1_s},
              CORAL_POS(kCoral_StowNoPiece, false),
              ALGAL_POS(kAlgae_L3Pick, false),
              WAIT{1_s},
              DRIVE(1PC, 100_in, START_Y - 30_in, 0_deg, 0_fps),
              DRIVE(1PC, 100_in, START_Y, 0_deg, 0_fps),
              ALGAL_POS(kAlgae_Net, true),
              WAIT{1_s},
          }} {}

LeaveAuto::LeaveAuto(
    RobotContainer& container, bool is_blue_side, bool is_left_side)
    : frc846::robot::GenericCommandGroup<RobotContainer, LeaveAuto, SEQUENCE>{
          container, AUTO_NAME("LEAVE"),
          SEQUENCE{
              START(158.5_in, START_Y, 180_deg),
              WAIT{0.25_s},
              DRIVE(LEAVE, 158.5_in, START_Y - 3_ft, 180_deg, 0_fps),
          }} {}