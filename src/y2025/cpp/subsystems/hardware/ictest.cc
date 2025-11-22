#include "subsystems/hardware/ictest.h"

#include <memory>

#include "frc846/control/base/motor_specs.h"
#include "frc846/wpilib/units.h"
#include "pdcsu.h"
#include "ports.h"

using namespace frc846::control;
using namespace frc846::control::config;
using namespace pdcsu::control;
using namespace pdcsu::util;
using namespace pdcsu::units;

ICTestSubsystem::ICTestSubsystem()
    : GenericSubsystem("ictest"),
      motor_configs_{},
      esc_{base::SPARK_MAX_NEO, motor_configs_} {
  // Register preferences for motor configuration
  RegisterPreference("motor_current_limit", 80_A);
  RegisterPreference("smart_current_limit", 60_A);
  RegisterPreference("voltage_compensation", 12_V);
  RegisterPreference("rotational_inertia", frc846::wpilib::unit_kg_m_sq{0.001});
  RegisterPreference("circuit_resistance", frc846::wpilib::unit_ohm{0.01});
  RegisterPreference("friction", 0.04);
}

ICTestSubsystem::~ICTestSubsystem() = default;

void ICTestSubsystem::Setup() {
  // Construct motor configs with all required fields from preferences
  motor_configs_ = MotorConstructionParameters{
      .can_id = ports::ictest_::kMotor_CANID,
      .inverted = false,
      .brake_mode = true,
      .motor_current_limit =
          GetPreferenceValue_unit_type<units::ampere_t>("motor_current_limit"),
      .smart_current_limit =
          GetPreferenceValue_unit_type<units::ampere_t>("smart_current_limit"),
      .voltage_compensation =
          GetPreferenceValue_unit_type<units::volt_t>("voltage_compensation"),
      .circuit_resistance =
          GetPreferenceValue_unit_type<frc846::wpilib::unit_ohm>(
              "circuit_resistance"),
      .rotational_inertia =
          GetPreferenceValue_unit_type<frc846::wpilib::unit_kg_m_sq>(
              "rotational_inertia"),
      .friction = GetPreferenceValue_double("friction"),
      .bus = "",
  };

  // Reconstruct ESC with proper configs (must be done before Setup)
  esc_ = HigherMotorController{base::SPARK_MAX_NEO, motor_configs_};
  esc_.Setup();
  esc_.EnableStatusFrames({kFaultFrame, kCurrentFrame});

  // Get motor specs for ICNOR
  auto motor_specs = base::MotorSpecificationPresets::get(base::SPARK_MAX_NEO);

  // Create BasePlant for ICNOR
  DefBLDC def_bldc(amp_t(motor_specs.stall_current.value()),
      amp_t(motor_specs.free_current.value()),
      nm_t(motor_specs.stall_torque.value()),
      rpm_t(motor_specs.free_speed.value()), 12_u_V);

  // Create load function (no external load for test)
  auto load_fn = [](radian_t theta, radps_t omega) -> nm_t {
    (void)theta;
    (void)omega;
    return 0_u_Nm;
  };

  // Create BasePlant
  BasePlant plant{def_bldc, kgm2_t(motor_configs_.rotational_inertia.value()),
      nm_t(motor_specs.stall_torque.value() * motor_configs_.friction),
      UnitDivision<nm_t, rpm_t>(0.0), load_fn, ms_t(20.0),
      ohm_t(motor_configs_.circuit_resistance.value())};

  // Create ICNOR position controller
  icnor_controller_ = std::make_unique<ICNORPositionControl>(plant);

  icnor_controller_->setConstraints(
      radps_t(motor_specs.free_speed.value() * 0.85),
      amp_t(GetPreferenceValue_unit_type<units::ampere_t>("motor_current_limit")
              .value()));

  icnor_controller_->setProjectionHorizon(4);
}

bool ICTestSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_.VerifyConnected(), ok, "Could not verify esc");
  return ok;
}

ICTestTarget ICTestSubsystem::ZeroTarget() const {
  ICTestTarget target;
  target.pos = 0_deg;
  return target;
}

ICTestReadings ICTestSubsystem::ReadFromHardware() {
  auto pos = esc_.GetPosition();
  auto vel = esc_.GetVelocity();
  ICTestReadings readings{
      units::degree_t(pos), units::degrees_per_second_t(vel)};

  // Graph position
  Graph("position", readings.pos);
  Graph("velocity", readings.vel);

  return readings;
}

void ICTestSubsystem::WriteToHardware(ICTestTarget target) {
  if (!icnor_controller_) { return; }

  // Get current state directly from hardware
  auto pos = esc_.GetPosition();
  auto vel = esc_.GetVelocity();
  radian_t current_pos = radian_t(pos.value());
  radps_t current_vel = radps_t(vel.value());

  // Get target
  radian_t target_pos = radian_t(target.pos.value());
  radps_t target_vel = 0_u_radps;

  // Get control output from ICNOR
  double output = icnor_controller_->getOutput(
      target_pos, target_vel, current_pos, current_vel);

  // Write as duty cycle
  esc_.WriteDC(output);
}