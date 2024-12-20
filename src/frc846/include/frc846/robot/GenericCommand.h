#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/base/Loggable.h"
#include "frc846/wpilib/time.h"

namespace frc846::robot {

template <typename RobotContainer, typename Subclass>
class GenericCommand : public frc2::CommandHelper<frc2::Command, Subclass>,
                       public frc846::base::Loggable {
public:
  GenericCommand(RobotContainer& container, std::string name)
      : Loggable{name}, container_{container} {
    frc2::Command::SetName(name);
    Log("Constructing instance of command {}.", name);
  }

  virtual ~GenericCommand() {
    Log("Destroying instance of command {}.", name());
  }

  virtual void OnInit() = 0;
  virtual void OnEnd(bool interrupted) = 0;
  virtual void Periodic() = 0;

  void Initialize() override final {
    Log("Command {} initialized.", name());
    OnInit();

    command_start_time_ = frc846::wpilib::CurrentFPGATime();
  }

  void End(bool interrupted) override final {
    units::millisecond_t total_time =
        frc846::wpilib::CurrentFPGATime() - command_start_time_;

    Log("Command {} {}. Took {} ms to complete. Avg periodic {} ms. Peak "
        "periodic {} ms.",
        name(), interrupted ? "interrupted" : "finished",
        total_time.to<double>(), avg_periodic_time_ / 1000_us,
        max_periodic_time_ / 1000_us);

    OnEnd(interrupted);
  }

  void Execute() override final {
    const units::microsecond_t start_time = frc846::wpilib::CurrentFPGATime();
    Periodic();
    const units::microsecond_t end_time = frc846::wpilib::CurrentFPGATime();

    const auto elapsed_time = end_time - start_time;

    if (elapsed_time > 1000_us) {
      Warn("Command {} periodic overrun. Took {} ms.", name(),
          elapsed_time / 1000_us);
    }

    avg_periodic_time_ =
        (avg_periodic_time_ * num_periodic_loops_ + elapsed_time) /
        (num_periodic_loops_ + 1);
    max_periodic_time_ = std::max(max_periodic_time_, elapsed_time);

    num_periodic_loops_++;
  }

protected:
  RobotContainer& container_;

private:
  units::microsecond_t avg_periodic_time_ = 0.0_us;
  int num_periodic_loops_ = 0;

  units::microsecond_t max_periodic_time_ = 0.0_us;

  units::millisecond_t command_start_time_ = 0.0_ms;
};

template <typename RobotContainer, typename Subclass,
    wpi::DecayedDerivedFrom<frc2::Command>... Commands>
class GenericCommandGroup
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, Subclass>,
      public frc846::base::Loggable {
public:
  GenericCommandGroup(
      RobotContainer& container, std::string name, Commands&&... commands)
      : Loggable{name}, container_{container} {
    frc2::Command::SetName(name);

    frc2::SequentialCommandGroup::AddCommands(start_command_addition);
    frc2::SequentialCommandGroup::AddCommands(
        std::forward<Commands>(commands)...);
    frc2::SequentialCommandGroup::AddCommands(end_command_addition);

    Log("Constructing instance of command group {}.", name);
  }

protected:
  RobotContainer& container_;
  units::millisecond_t command_start_time_ = 0.0_ms;

private:
  frc2::InstantCommand end_command_addition{[&] {
    Log("Command group ending. Took {} ms to complete.",
        (frc846::wpilib::CurrentFPGATime() - command_start_time_).to<double>());
  }};

  frc2::InstantCommand start_command_addition{[&] {
    Log("Command group starting.");
    command_start_time_ = frc846::wpilib::CurrentFPGATime();
  }};
};

}  // namespace frc846::robot
