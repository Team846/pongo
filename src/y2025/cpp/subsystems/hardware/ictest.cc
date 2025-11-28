#include "subsystems/hardware/ictest.h"

#include <frc/Filesystem.h>

#include <memory>
#include <string>

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
    : GenericSubsystem("ictest"), esc_{base::SPARK_MAX_NEO} {
  RegisterPreference("motor_current_limit", 40_A);
  RegisterPreference("smart_current_limit", 30_A);
  RegisterPreference("voltage_compensation", 12_V);
  RegisterPreference("rotational_inertia", 2.2 * 0.03 * 0.03);  // kg·m²
  RegisterPreference("circuit_resistance", 0.01);               // ohms
  RegisterPreference("friction", 0.04);
  RegisterPreference("num_motors", 1);
  RegisterPreference("viscous_damping", 0.0);
}

ICTestSubsystem::~ICTestSubsystem() = default;

void ICTestSubsystem::Setup() {
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
      .circuit_resistance = frc846::wpilib::unit_ohm{GetPreferenceValue_double(
          "circuit_resistance")},
      .rotational_inertia =
          frc846::wpilib::unit_kg_m_sq{
              GetPreferenceValue_double("rotational_inertia")},
      .friction = GetPreferenceValue_double("friction"),
      .bus = "",
  };

  esc_.Setup(motor_configs_);
  esc_.EnableStatusFrames(
      {kFaultFrame, kCurrentFrame, kPositionFrame, kVelocityFrame});
  esc_.SetPosition(0_deg);

  auto motor_specs = base::MotorSpecificationPresets::get(base::SPARK_MAX_NEO);

  DefBLDC def_bldc(amp_t(motor_specs.stall_current.value()),
      amp_t(motor_specs.free_current.value()),
      nm_t(motor_specs.stall_torque.value()),
      rpm_t(motor_specs.free_speed.value()),
      volt_t(motor_configs_.voltage_compensation.value()));

  int num_motors = GetPreferenceValue_int("num_motors");
  scalar_t gear_ratio = scalar_t(0.33);
  kgm2_t inertia = kgm2_t(motor_configs_.rotational_inertia.value());
  nm_t friction =
      nm_t(motor_specs.stall_torque.value() * motor_configs_.friction);
  double viscous_damping_val = GetPreferenceValue_double("viscous_damping");
  UnitDivision<nm_t, rpm_t> viscous_damping =
      UnitDivision<nm_t, rpm_t>(viscous_damping_val);
  ms_t control_period = ms_t(20.0);
  ohm_t circuit_res = ohm_t(motor_configs_.circuit_resistance.value());

  auto load_fn_0 = [](radian_t theta, radps_t omega) -> nm_t {
    (void)theta;
    (void)omega;
    return 0_u_Nm;
  };

  angular_sys_ =
      std::make_unique<DefArmSys>(def_bldc, num_motors, gear_ratio, load_fn_0,
          inertia, friction, viscous_damping, control_period, circuit_res);

  icnor_controller_ = std::make_unique<ICNORPositionControl>(*angular_sys_);

  icnor_controller_->setConstraints(
      radps_t(motor_specs.free_speed.value() * 0.85),
      amp_t(motor_configs_.smart_current_limit.value()));

  icnor_controller_->setProjectionHorizon(3);

  // Create and attach ICNOR learner
  std::string learner_path =
      frc::filesystem::GetDeployDirectory() + "/ictest.iclearn";
  icnor_learner_ = std::make_shared<ICNORLearner>(learner_path);
  icnor_controller_->attachLearner(icnor_learner_);
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
  if (!angular_sys_) { return ICTestReadings{0_deg, 0_deg_per_s}; }

  auto pos_native = esc_.GetPosition();
  auto vel_native = esc_.GetVelocity();

  radian_t pos_real = angular_sys_->toReal(radian_t(pos_native.value()));
  radps_t vel_real = angular_sys_->toReal(radps_t(vel_native.value()));

  ICTestReadings readings{units::degree_t(pos_real.value()),
      units::degrees_per_second_t(vel_real.value())};

  Graph("position", readings.pos);
  Graph("velocity", readings.vel);

  return readings;
}

void ICTestSubsystem::WriteToHardware(ICTestTarget target) {
  if (!icnor_controller_ || !angular_sys_) { return; }

  auto pos_native = esc_.GetPosition();
  auto vel_native = esc_.GetVelocity();
  radian_t current_pos_native = radian_t(pos_native.value());
  radps_t current_vel_native = radps_t(vel_native.value());

  radian_t target_pos_real = radian_t(target.pos.value());
  radian_t target_pos_native = angular_sys_->toNative(target_pos_real);
  radps_t target_vel_native = 0_u_radps;

  double output = icnor_controller_->getOutput(target_pos_native,
      target_vel_native, current_pos_native, current_vel_native);

  esc_.WriteDC(output);
}