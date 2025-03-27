#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

#define ADD_AUTO_VARIANTS(class_name, name_string) \
  AddAuto(std::string(name_string) + "/R/L",       \
      new class_name{container_, false, true});    \
  AddAuto(std::string(name_string) + "/R/R",       \
      new class_name{container_, false, false});   \
  AddAuto(std::string(name_string) + "/B/L",       \
      new class_name{container_, true, true});     \
  AddAuto(std::string(name_string) + "/B/R",       \
      new class_name{container_, true, false});

class FourAndPickAuto
    : public frc846::robot::GenericCommandGroup<RobotContainer, FourAndPickAuto,
          frc2::SequentialCommandGroup> {
public:
  FourAndPickAuto(
      RobotContainer& container, bool is_blue_side, bool is_left_side);
};

class SimScorePoints : public frc846::robot::GenericCommandGroup<RobotContainer,
                           SimScorePoints, frc2::SequentialCommandGroup> {
public:
  SimScorePoints(
      RobotContainer& container, bool is_blue_side, bool is_left_side);
};

class OnePieceAndNetAuto
    : public frc846::robot::GenericCommandGroup<RobotContainer,
          OnePieceAndNetAuto, frc2::SequentialCommandGroup> {
public:
  OnePieceAndNetAuto(
      RobotContainer& container, bool is_blue_side, bool is_left_side);
};

class LeaveAuto : public frc846::robot::GenericCommandGroup<RobotContainer,
                      LeaveAuto, frc2::SequentialCommandGroup> {
public:
  LeaveAuto(RobotContainer& container, bool is_blue_side, bool is_left_side);
};
