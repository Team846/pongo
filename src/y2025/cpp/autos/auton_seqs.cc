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
#define MAX_DECEL_3PC 20_fps_sq
#define MAX_VEL_3PC 15_fps

#define MAX_ACCEL_1PC 24_fps_sq
#define MAX_DECEL_1PC 24_fps_sq
#define MAX_VEL_1PC 13_fps
#define MAX_ACCEL_1PCS 11_fps_sq
#define MAX_DECEL_1PCS 11_fps_sq
#define MAX_VEL_1PCS 9_fps

#define MAX_ACCEL_LEAVE 10_fps_sq
#define MAX_DECEL_LEAVE 10_fps_sq
#define MAX_VEL_LEAVE 5_fps

#define MAX_ACCEL_SIMTEST 30_fps_sq
#define MAX_DECEL_SIMTEST 30_fps_sq
#define MAX_VEL_SIMTEST 15_fps

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

#define START2(x, y, start_bearing)                              \
  INSTANT {                                                      \
    [&, blue = is_blue_side, left = is_left_side]() {            \
      FPT start_point{{x, y}, start_bearing, 0_fps};             \
      start_point = start_point.mirror(blue).mirrorOnlyX(!left); \
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

#define SOURCELOC_PRE MKPT(34.203_in, 69.432_in, 53.5_deg, 0_fps)
#define SOURCELOC MKPT(17.03_in, 50.532_in, 53.5_deg, 0_fps)

#define FPC_EXPECTED_START_UF \
  FPT { {50.5_in, 271.3_in}, 140_deg, 0_fps }
#define SIM_EXP_START_UF \
  FPT { {20_in, 20_in}, 0_deg, 0_fps }

#define FPC_SIM_START()                                                      \
  INSTANT {                                                                  \
    [&, blue = is_blue_side, left = is_left_side] {                          \
      if (!frc::RobotBase::IsSimulation()) return;                           \
      FPT exp_start = FPC_EXPECTED_START_UF.mirror(blue).mirrorOnlyX(!left); \
      container.drivetrain_.SetPosition(exp_start.point);                    \
      container.drivetrain_.SetOdomBearing(exp_start.bearing);               \
      Log("FPC Simulated Auto Start");                                       \
    }                                                                        \
  }

#define SIM_TEST_START()                                                \
  INSTANT {                                                             \
    [&, blue = is_blue_side, left = is_left_side] {                     \
      if (!frc::RobotBase::IsSimulation()) return;                      \
      FPT exp_start = SIM_EXP_START_UF.mirror(blue).mirrorOnlyX(!left); \
      container.drivetrain_.SetPosition(exp_start.point);               \
      container.drivetrain_.SetOdomBearing(exp_start.bearing);          \
      Log("Simulated Auto Start");                                      \
    }                                                                   \
  }

#define DRIVE_TO_SOURCE(auto_name)                                       \
  frc2::ParallelDeadlineGroup {                                          \
    frc846::robot::swerve::DriveToPointCommand{&(container.drivetrain_), \
        SOURCELOC_PRE, MAX_VEL_##auto_name, MAX_ACCEL_##auto_name,       \
        MAX_DECEL_##auto_name, true},                                    \
        CORAL_POS(kCoral_StowNoPiece, false)                             \
  }

#define LOCK_TO_SOURCE()                                          \
  frc2::ParallelDeadlineGroup {                                   \
    WAIT_FOR_PIECE(), frc846::robot::swerve::LockToPointCommand { \
      &(container.drivetrain_), SOURCELOC                         \
    }                                                             \
  }

#define SMART_LOCK_SOURCE()                                               \
  frc2::ParallelDeadlineGroup {                                           \
    WAIT_FOR_PIECE(), SEQUENCE {                                          \
      frc2::ParallelDeadlineGroup{WAIT{2.25_s}, LOCK_TO_SOURCE()},        \
          DRIVE_TO_SOURCE(3PC), WAIT{0.5_s},                              \
          PARALLEL_DEADLINE(WAIT{0.13_s}, CORAL_POS(kCoral_FLICK, true)), \
          PARALLEL_DEADLINE(                                              \
              LOCK_TO_SOURCE(), CORAL_POS(kCoral_StowNoPiece, false)),    \
    }                                                                     \
  }

#define DRIVE_TO_REEF(auto_name, number_on_right, isRetry)          \
  ReefAutoAutoAlignCommand {                                        \
    container, number_on_right, is_blue_side, is_left_side, isRetry \
  }

#define WAIT4REEF()                                                         \
  frc2::WaitUntilCommand {                                                  \
    [&] {                                                                   \
      if (frc::RobotBase::IsSimulation()) return false;                     \
      return container.coral_ss_.coral_end_effector.GetReadings().see_reef; \
    }                                                                       \
  }

#define WAIT4REEF_1PC()                                                     \
  frc2::WaitUntilCommand {                                                  \
    [&] {                                                                   \
      if (frc::RobotBase::IsSimulation()) return true;                      \
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
      if (frc::RobotBase::IsSimulation()) return true;        \
      return container.coral_ss_.GetReadings().piece_entered; \
    }                                                         \
  }

#define DRIVE_SCORE_REEF_3PC(reefNum)                                       \
  PARALLEL_DEADLINE(WAIT(0.125_s), CORAL_POS(kCoral_StowWithPiece, false)), \
      PARALLEL_DEADLINE(DRIVE_TO_REEF(3PC, reefNum, false),                 \
          SEQUENCE(WAIT(1.75_s), CORAL_POS(kCoral_ScoreL4, false))),        \
      CORAL_POS(kCoral_ScoreL4, false),                                     \
      PARALLEL_RACE(WAIT4REEF(), WAIT(0.75_s)),                             \
      PARALLEL_RACE(WAIT4REEF(), DRIVE_TO_REEF(3PC, reefNum, true)),        \
      CORAL_POS(kCoral_ScoreL4, true), WAIT {                               \
    0.25_s                                                                  \
  }

#define __AUTO__(codeName, stringName)                                 \
  codeName::codeName(                                                  \
      RobotContainer& container, bool is_blue_side, bool is_left_side) \
      : frc846::robot::GenericCommandGroup<RobotContainer, codeName,   \
            SEQUENCE> {                                                \
    container, AUTO_NAME(stringName),

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

__AUTO__(FourAndPickAuto, "5PC")
SEQUENCE {  // START(158.5_in - 73.25_in, START_Y, 180_deg),
  // WAIT{0.25_s},
  FPC_SIM_START(), DRIVE_SCORE_REEF_3PC(11), DRIVE_TO_SOURCE(3PC),
      SMART_LOCK_SOURCE(), DRIVE_SCORE_REEF_3PC(is_left_side ? 9 : 8),
      DRIVE_TO_SOURCE(3PC), SMART_LOCK_SOURCE(),
      DRIVE_SCORE_REEF_3PC(is_left_side ? 8 : 9),
      ALGAL_POS(kAlgae_L2Pick, false), WAIT{0.5_s}, DRIVE_TO_SOURCE(3PC),
}
}
{}

__AUTO__(OnePieceAndNetAuto, "1PCN")
SEQUENCE {
  START2(158.5_in, START_Y + 1.5_in, 180_deg), WAIT{0.5_s},
      DRIVE(1PC, 158.5_in + 1.5_in, 231.975_in, 180_deg, 0_fps),
      PARALLEL_DEADLINE(
          CORAL_POS(kCoral_ScoreL4, false), ALGAL_POS(kAlgae_L2Pick, false)),
      WAIT4REEF_1PC(), CORAL_POS(kCoral_ScoreL4, true), WAIT{0.25_s},
      DRIVE(1PC, 158.5_in + 1.5_in, 229.75_in, 180_deg, 0_fps), WAIT{1.0_s},
      CORAL_POS(kCoral_StowNoPiece, false), WAIT{1.0_s},
      DRIVE(1PC, 135_in, START_Y - 35_in, 0_deg, 0_fps),
      DRIVE(1PCS, 110_in, START_Y + 16_in, 0_deg, 0_fps),
      ALGAL_POS(kAlgae_Net, false), WAIT{0.5_s}, ALGAL_POS(kAlgae_Net, true),
      WAIT{1.0_s}, DRIVE(1PCS, 100_in, START_Y - 40_in, 0_deg, 0_fps),
      ALGAL_POS(kAlgae_Stow, false), WAIT{4.0_s},
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

__AUTO__(SimTestAuto, "SIMTEST")
SEQUENCE {
  SIM_TEST_START(), DRIVE(SIMTEST, 42_in, 51.66_in, 0_deg, 4_fps),
      DRIVE(SIMTEST, 49_in, 63.43_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 57_in, 70.01_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 65_in, 73.54_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 75_in, 75_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 85_in, 76.45_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 94_in, 80.60_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 101_in, 86.56_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 107_in, 95.82_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 110_in, 110_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 108_in, 121.66_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 101_in, 133.43_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 89_in, 142.07_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 75_in, 145_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 61_in, 142.07_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 49_in, 133.43_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 42_in, 121.66_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 40_in, 110_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 43_in, 95.82_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 49_in, 86.56_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 56_in, 80.60_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 65_in, 76.45_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 75_in, 75_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 85_in, 73.54_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 93_in, 70.01_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 101_in, 63.43_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 108_in, 51.66_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 110_in, 40_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 108_in, 28.34_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 101_in, 16.57_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 93_in, 9.99_in, 0_deg, 6_fps),
      DRIVE(SIMTEST, 85_in, 6.46_in, 0_deg, 4_fps),
      DRIVE(SIMTEST, 75_in, 5_in, 0_deg, 0_fps),
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