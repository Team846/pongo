#pragma once

#include "frc846/robot/GenericSubsystem.h"

namespace frc846::robot {

class GenericRobotContainer : public frc846::base::Loggable {
public:
  GenericRobotContainer() : frc846::base::Loggable{"robot_container"} {}

  void RegisterSubsystemGroupA(
      std::initializer_list<std::pair<frc846::robot::SubsystemBase*, bool>>
          subsystems) {
    for (auto& [subsystem, init] : subsystems) {
      if (init) {
        subsystem->Init();
        group_a_subsystems_.push_back(subsystem);
        all_subsystems_.push_back(subsystem);
      }
    }
  }

  void RegisterSubsystemGroupB(
      std::initializer_list<std::pair<frc846::robot::SubsystemBase*, bool>>
          subsystems) {
    for (auto& [subsystem, init] : subsystems) {
      if (init) {
        subsystem->Init();
        group_b_subsystems_.push_back(subsystem);
        all_subsystems_.push_back(subsystem);
      }
    }
  }

  void RegisterSubsystemGroupAB(
      std::initializer_list<std::pair<frc846::robot::SubsystemBase*, bool>>
          subsystems) {
    for (auto& [subsystem, init] : subsystems) {
      if (init) {
        subsystem->Init();
        group_a_subsystems_.push_back(subsystem);
        group_b_subsystems_.push_back(subsystem);
        all_subsystems_.push_back(subsystem);
      }
    }
  }

  void UpdateReadings() {
    if (read_counter % 2 == 0) {
      for (auto subsystem : group_a_subsystems_) {
        subsystem->UpdateReadings();
      }
    } else {
      for (auto subsystem : group_b_subsystems_) {
        subsystem->UpdateReadings();
      }
    }
    read_counter++;
  }

  void UpdateHardware() {
    if (write_counter % 2 == 0) {
      for (auto subsystem : group_a_subsystems_) {
        subsystem->UpdateHardware();
      }
    } else {
      for (auto subsystem : group_b_subsystems_) {
        subsystem->UpdateHardware();
      }
    }
    write_counter++;
  }

  void Setup() {
    for (auto subsystem : all_subsystems_) {
      subsystem->Setup();
    }
  }

  void ZeroTargets() {
    for (auto subsystem : all_subsystems_) {
      subsystem->SetTargetZero();
    }
  }

  void VerifyHardware() {
    for (auto subsystem : all_subsystems_) {
      subsystem->VerifyHardware();
    }
  }

private:
  std::vector<frc846::robot::SubsystemBase*> all_subsystems_{};
  std::vector<frc846::robot::SubsystemBase*> group_a_subsystems_{};
  std::vector<frc846::robot::SubsystemBase*> group_b_subsystems_{};

  unsigned int read_counter;
  unsigned int write_counter;
};

}  // namespace frc846::robot
