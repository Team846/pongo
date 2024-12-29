#pragma once

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>

#include "frc846/base/Loggable.h"

namespace frc846::robot {

#define FRC846_VERIFY(expr, ok, fail_msg)       \
  if (!(expr)) {                                \
    ok = false;                                 \
    Error("Verification failed: {}", fail_msg); \
  }

// Non-templated subsystem base class.
class SubsystemBase : public frc846::base::Loggable {
public:
  SubsystemBase(std::string name) : Loggable{name} {}
  SubsystemBase(Loggable parent, std::string name) : Loggable{parent, name} {}

  virtual ~SubsystemBase() = default;

  virtual void Setup() = 0;

  virtual void UpdateReadings() = 0;
  virtual void UpdateHardware() = 0;
  virtual bool VerifyHardware() = 0;
  virtual void SetTargetZero() = 0;
};

// Base class for robot subsystems.
template <class Readings, class Target>
class GenericSubsystem : public frc2::SubsystemBase, public SubsystemBase {
public:
  // Construct a new subsystem.
  explicit GenericSubsystem(std::string name)
      : frc846::robot::SubsystemBase{name} {}

  // Construct a subsystem as a child of another subsystem.
  explicit GenericSubsystem(const Loggable& parent, std::string name)
      : frc846::robot::SubsystemBase{parent, name} {}

  bool is_initialized() { return init_; }

  GenericSubsystem(const GenericSubsystem&) = delete;
  GenericSubsystem& operator=(const GenericSubsystem&) = delete;

  virtual ~GenericSubsystem() { Warn("Destroying subsystem"); };

  // Initializer function for RobotContainer use only.
  void Init() {
    SetName(name());
    Log("Initializing subsystem");
    init_ = true;
  }

  /*
  InitByParent()

  Initializer function to be called by a parent subsystem only. Will not
  register with WPILib.
  */
  void InitByParent() {
    Log("Initializing subsystem (by parent)");
    init_ = true;
  }

private:
  bool init_;

public:
  // Get the zero state target.
  virtual Target ZeroTarget() const = 0;

  // Fetches new readings and update subsystem readings state.
  void UpdateReadings() override final {
    if (is_initialized()) {
      readings_ = ReadFromHardware();
    } else {
      readings_ = Readings{};
    }
  }

  // Writes to subsystem hardware with the latest target output.
  void UpdateHardware() override final {
    if (is_initialized()) WriteToHardware(target_);
  }

  virtual bool VerifyHardware() override = 0;

  // Get the latest readings.
  Readings GetReadings() const { return readings_; };

  // Set the subystem target state.
  void SetTarget(Target target) { target_ = target; }

  // Set the subsystem to its zero state.
  void SetTargetZero() override { target_ = ZeroTarget(); }

  auto GetTarget() { return target_; }

private:
  Readings readings_;
  Target target_;

protected:
  // Fetches and return new readings.
  virtual Readings ReadFromHardware() = 0;

  // Writes output to hardware.
  virtual void WriteToHardware(Target target) = 0;
};

}  // namespace frc846::robot
