#pragma once

#include "frc846/robot/GenericSubsystem.h"

namespace frc846::robot {

class GenericRobotContainer : public frc846::base::Loggable {
 public:
  GenericRobotContainer() : frc846::base::Loggable{"robot_container"} {}

  void RegisterSubsystems(
      std::vector<frc846::robot::SubsystemBase*> subsystems) {
    all_subsystems_ = subsystems;
  }

  void UpdateReadings() {
    for (auto subsystem : all_subsystems_) {
      subsystem->UpdateReadings();
    }
  }

  void UpdateHardware() {
    for (auto subsystem : all_subsystems_) {
      subsystem->UpdateHardware();
    }
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
};

}  // namespace frc846::robot
