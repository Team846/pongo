#include "autos/auton_seqs.h"

#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include "commands/general/algal_position_command.h"
#include "commands/general/coral_position_command.h"
#include "commands/general/reef_auto_auto_align.h"
#include "frc846/robot/swerve/aim_command.h"
#include "frc846/robot/swerve/drive_to_point_command.h"
#include "frc846/robot/swerve/lock_to_point_command.h"
#include "frc846/robot/swerve/wait_until_close.h"
#include "reef.h"

/************************
AUTONOMOUS HELPER MACROS

START DEFINE MACROS
*************************/

using INSTANT = frc2::InstantCommand;
using SEQUENCE = frc2::SequentialCommandGroup;
using WAIT = frc2::WaitCommand;

using FPT = frc846::math::FieldPoint;

#define MAX_ACCEL_3PC 25_fps_sq
#define MAX_DECEL_3PC 8_fps_sq
#define MAX_VEL_3PC 8_fps

#define MAX_ACCEL_1PC 17_fps_sq
#define MAX_DECEL_1PC 17_fps_sq
#define MAX_VEL_1PC 13_fps
#define MAX_ACCEL_1PCS 7_fps_sq
#define MAX_DECEL_1PCS 7_fps_sq
#define MAX_VEL_1PCS 8_fps

#define MAX_ACCEL_LEAVE 10_fps_sq
#define MAX_DECEL_LEAVE 10_fps_sq
#define MAX_VEL_LEAVE 5_fps

#define START_Y (298.5_in - 16.5_in)

#define PARALLEL_DEADLINE(deadline, parallel) \
  frc2::ParallelDeadlineGroup { deadline, parallel }

#define PARALLEL_RACE(action1, action2) \
  frc2::ParallelRaceGroup { action1, action2 }

#define SEQUENCE(action1, action2) \
  frc2::SequentialCommandGroup { action1, action2 }

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
      container.drivetrain_.SetBearing(start_point.bearing);     \
      container.drivetrain_.UpdateReadings();                    \
      container.drivetrain_.SetPosition(                         \
          container.drivetrain_.GetReadings().april_point);      \
      Log("Auto Start");                                         \
    }                                                            \
  }
#define START2(x, y, start_bearing)                              \
  INSTANT {                                                      \
    [&, blue = is_blue_side, left = is_left_side]() {            \
      FPT start_point{{x, y}, start_bearing, 0_fps};             \
      start_point = start_point.mirror(blue).mirrorOnlyX(!left); \
      container.drivetrain_.SetBearing(start_point.bearing);     \
      container.drivetrain_.UpdateReadings();                    \
      container.drivetrain_.SetPosition(                         \
          {start_point.point[0], start_point.point[1]});         \
      Log("Auto Start");                                         \
    }                                                            \
  }
#define DRIVE(auto_name, x, y, bearing, final_velocity)                   \
  frc846::robot::swerve::DriveToPointCommand {                            \
    &(container.drivetrain_), MKPT(x, y, bearing, final_velocity),        \
        MAX_VEL_##auto_name, MAX_ACCEL_##auto_name, MAX_DECEL_##auto_name \
  }
#define DRIVE_TO_SOURCE(auto_name)                                       \
  frc2::ParallelDeadlineGroup {                                          \
    frc846::robot::swerve::DriveToPointCommand{&(container.drivetrain_), \
        MKPT(29.5_in, 53.75_in, 53.5_deg, 0_fps), MAX_VEL_##auto_name,   \
        MAX_ACCEL_##auto_name, MAX_DECEL_##auto_name},                   \
        CORAL_POS(kCoral_StowNoPiece, false)                             \
  }

#define DRIVE_TO_SOURCE_END(auto_name)                                   \
  frc2::ParallelDeadlineGroup {                                          \
    frc846::robot::swerve::DriveToPointCommand{&(container.drivetrain_), \
        MKPT(48.25_in, 25_in, 53.5_deg, 0_fps), MAX_VEL_##auto_name,     \
        MAX_ACCEL_##auto_name, MAX_DECEL_##auto_name},                   \
        CORAL_POS(kCoral_StowNoPiece, false)                             \
  }

#define DRIVE_TO_REEF(auto_name, number_on_right)          \
  ReefAutoAutoAlignCommand {                               \
    container, number_on_right, is_blue_side, is_left_side \
  }

#define DRIVE_TO_REEF_NOAT(auto_name, number_on_right)                     \
  frc846::robot::swerve::DriveToPointCommand {                             \
    &(container.drivetrain_),                                              \
        ReefProvider::getReefScoringLocations(false)[number_on_right]      \
            .mirror(is_blue_side)                                          \
            .mirrorOnlyX(!is_left_side),                                   \
        MAX_VEL_##auto_name, MAX_ACCEL_##auto_name, MAX_DECEL_##auto_name, \
        true                                                               \
  }

#define WAIT4REEF()                                                         \
  frc2::WaitUntilCommand {                                                  \
    [&] {                                                                   \
      return container.coral_ss_.coral_end_effector.GetReadings().see_reef; \
    }                                                                       \
  }

#define AIM(bearing)                                                     \
  frc846::robot::swerve::AimCommand {                                    \
    &(container.drivetrain_), bearing + (is_blue_side ? 180_deg : 0_deg) \
  }

#define ALGAL_POS(where, score) \
  AlgalPositionCommand { container, where, score }
#define CORAL_POS(where, score) \
  CoralPositionCommand { container, where, score }

#define WAIT_FOR_PIECE()                                      \
  frc2::WaitUntilCommand {                                    \
    [&] {                                                     \
      return container.coral_ss_.GetReadings().piece_entered; \
    }                                                         \
  }

#define DRIVE_SCORE_REEF_3PC(reefNum)                                         \
  DRIVE_TO_REEF(3PC, reefNum), CORAL_POS(kCoral_ScoreL4, false), WAIT{0.5_s}, \
      PARALLEL_RACE(WAIT4REEF(), WAIT(1.25_s)),                               \
      CORAL_POS(kCoral_ScoreL4, true), WAIT {                                 \
    0.25_s                                                                    \
  }

#define __AUTO__(codeName, stringName)                                 \
  codeName::codeName(                                                  \
      RobotContainer& container, bool is_blue_side, bool is_left_side) \
      : frc846::robot::GenericCommandGroup<RobotContainer, codeName,   \
            SEQUENCE> {                                                \
    container, AUTO_NAME(stringName),

// TODO: Use AprilTags for start point?

/***********************
AUTONOMOUS HELPER MACROS

END DEFINE MACROS
************************/

/************************
| ---------------------- |
|  AUTONOMOUS SEQUENCES  |
|                        |
| START DEFINE SEQUENCES |
| ---------------------- |
*************************/

__AUTO__(FourAndPickAuto, "5PC") SEQUENCE {
  // START(158.5_in - 73.25_in, START_Y, 180_deg),
  // WAIT{0.25_s},
  DRIVE_SCORE_REEF_3PC(11), DRIVE_TO_SOURCE(3PC), WAIT_FOR_PIECE(),
      DRIVE_SCORE_REEF_3PC(8), DRIVE_TO_SOURCE(3PC), WAIT_FOR_PIECE(),
      DRIVE_SCORE_REEF_3PC(9), DRIVE_TO_SOURCE_END(3PC), WAIT_FOR_PIECE(),
      DRIVE_SCORE_REEF_3PC(6),
}
}
{}

__AUTO__(OnePieceAndNetAuto, "1PCN")
SEQUENCE {
  START2(158.5_in + 1.5_in, START_Y, 180_deg), WAIT{0.5_s},
      DRIVE_TO_REEF_NOAT(1PC, 0), CORAL_POS(kCoral_ScoreL4, false), WAIT4REEF(),
      PARALLEL_DEADLINE(
          SEQUENCE(SEQUENCE(CORAL_POS(kCoral_ScoreL4, true), WAIT{0.5_s}),
              CORAL_POS(kCoral_StowNoPiece, false)),
          ALGAL_POS(kAlgae_L2Pick, false)),
      WAIT{1.0_s}, DRIVE(1PC, 100_in, START_Y - 48_in, 0_deg, 0_fps),
      DRIVE(1PCS, 100_in, START_Y + 16_in, 0_deg, 0_fps),
      ALGAL_POS(kAlgae_Net, false), WAIT{0.5_s}, ALGAL_POS(kAlgae_Net, true),
      WAIT{1.0_s}, DRIVE(1PCS, 100_in, START_Y - 40_in, 0_deg, 0_fps),
      ALGAL_POS(kAlgae_Stow, false),
}
}
{}

__AUTO__(LeaveAuto, "LEAVE")
SEQUENCE {
  START2(158.5_in, START_Y, 180_deg), WAIT{0.25_s},
      DRIVE(LEAVE, 158.5_in, START_Y - 3_ft, 180_deg, 0_fps),
}
}
{}

/***********************
| --------------------- |
| AUTONOMOUS SEQUENCES  |
|                       |
| END DEFINE SEQUENCES  |
| --------------------- |
************************/