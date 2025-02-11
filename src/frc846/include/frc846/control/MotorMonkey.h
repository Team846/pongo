#pragma once

#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <vector>

#include "frc846/base/Loggable.h"
#include "frc846/control/base/motor_control_base.h"
#include "frc846/control/base/motor_gains.h"
#include "frc846/control/base/motor_specs.h"
#include "frc846/control/calculators/CurrentTorqueCalculator.h"
#include "frc846/control/config/construction_params.h"
#include "frc846/control/config/status_frames.h"
#include "frc846/control/hardware/TalonFX_interm.h"
#include "frc846/math/DoubleSyncBuffer.h"

#define CONTROLLER_REGISTRY_SIZE 64

namespace frc846::control {

/*
MotorMonkey

A class that provides higher level management for all motor controllers. Manages
CAN utilization as well as power.
*/
class MotorMonkey {
public:
  /*
  Setup()

  Sets up the preferences used by MotorMonkey.
  */
  static void Setup();

  /*
  Tick()

  Updates all motor controllers. Should be called each loop.
  */
  static void Tick(bool disabled);

  /*
  WriteMessages()

  Writes all messages in the message queue to the motor controllers. Also,
  dynamically manages current draw and drops redundant messages.
  Returns predicted total current draw.
  */
  static units::ampere_t WriteMessages(units::ampere_t max_draw);

  /*
  ConstructController()

  Constructs a motor controller and returns a slot ID for it. This slot ID is
  used to refer to the motor controller in future calls.
  */
  static size_t ConstructController(frc846::control::base::MotorMonkeyType type,
      frc846::control::config::MotorConstructionParameters params);

  /*
  EnableStatusFrames()

  Enables specific status frames for a motor controller. Disables all others.
  */
  static void EnableStatusFrames(size_t slot_id,
      std::vector<frc846::control::config::StatusFrame> frames,
      units::millisecond_t faults_ms = 20_ms,
      units::millisecond_t velocity_ms = 20_ms,
      units::millisecond_t encoder_position_ms = 20_ms,
      units::millisecond_t analog_position_ms = 20_ms);

  static void OverrideStatusFramePeriod(size_t slot_id,
      frc846::control::config::StatusFrame frame, units::millisecond_t period);

  /*
  GetBatteryVoltage()

  Returns the battery voltage.
  */
  static units::volt_t GetBatteryVoltage();

  /*
  SetLoad()

  Sets the load on a motor controller. This is used for calculations.
  */
  static void SetLoad(size_t slot_id, units::newton_meter_t load);

  static void SetGains(size_t slot_id, frc846::control::base::MotorGains gains);

  static void WriteDC(size_t slot_id, double duty_cycle);
  /*
  WriteVelocity()

  Writes a velocity setpoint to the motor controller. PID calculations performed
  onboard the motor controller.
  */
  static void WriteVelocity(
      size_t slot_id, units::radians_per_second_t velocity);
  /*
  WritePosition()

  Writes a position setpoint to the motor controller. PID calculations
  performed onboard the motor controller.
  */
  static void WritePosition(size_t slot_id, units::radian_t position);

  /*
  ZeroEncoder()

  Sets the position of the encoder to the specified value.
  */
  static void ZeroEncoder(size_t slot_id, units::radian_t position);

  static units::radians_per_second_t GetVelocity(size_t slot_id);
  static units::radian_t GetPosition(size_t slot_id);
  static units::ampere_t GetCurrent(size_t slot_id);

  static void ConfigForwardLimitSwitch(size_t slot_id, bool stop_motor,
      frc846::control::base::LimitSwitchDefaultState type);
  static void ConfigReverseLimitSwitch(size_t slot_id, bool stop_motor,
      frc846::control::base::LimitSwitchDefaultState type);

  static bool GetForwardLimitSwitchState(size_t slot_id);
  static bool GetReverseLimitSwitchState(size_t slot_id);

  static void SetSoftLimits(size_t slot_id, units::radian_t forward_limit,
      units::radian_t reverse_limit);

  static std::string_view parseError(
      frc846::control::hardware::ControllerErrorCodes err);

  static bool VerifyConnected();

private:
  static void RecalculateMaxDraw();

  static frc846::base::Loggable loggable_;

  static size_t slot_counter_;
  static std::map<size_t, frc846::control::base::MotorMonkeyType>
      slot_id_to_type_;
  static std::map<size_t, bool> slot_id_to_sim_;

  static frc846::control::hardware::IntermediateController*
      controller_registry[CONTROLLER_REGISTRY_SIZE];

  static frc846::control::base::MotorGains
      gains_registry[CONTROLLER_REGISTRY_SIZE];
  static units::newton_meter_t load_registry[CONTROLLER_REGISTRY_SIZE];

  static frc846::wpilib::unit_ohm
      circuit_resistance_registry[CONTROLLER_REGISTRY_SIZE];

  static units::volt_t battery_voltage;
  static units::volt_t last_disabled_voltage;

  static frc846::math::DoubleSyncBuffer sync_buffer;

  static units::ampere_t max_draw_;

  struct MotorMessage {
    enum class Type { DC, Position, Velocity };
    size_t slot_id;
    Type type;
    std::variant<double, units::radian_t, units::radians_per_second_t> value;
  };

  static std::queue<MotorMessage> control_messages;

  static int num_loops_last_brown;
};

}  // namespace frc846::control