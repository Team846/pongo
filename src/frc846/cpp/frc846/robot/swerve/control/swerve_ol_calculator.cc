#include "frc846/robot/swerve/control/swerve_ol_calculator.h"

namespace frc846::robot::swerve::control {

SwerveOpenLoopCalculatorOutput SwerveOpenLoopCalculator::calculate(
    SwerveOpenLoopCalculatorInputs inputs) {
  std::pair<double, double> kModuleLocationSigns[4] = {
      {+0.5, +0.5},  // FR
      {-0.5, +0.5},  // FL
      {-0.5, -0.5},  // BL
      {+0.5, -0.5},  // BR
  };

  std::array<frc846::math::VectorND<units::feet_per_second, 2>, 4>
      module_targets;

  units::inch_t radius = units::math::hypot(
      constants_.wheelbase_horizontal_dim, constants_.wheelbase_forward_dim);

  auto calculate_module_targets = [&](units::radians_per_second_t rotation) {
    for (int i = 0; i < 4; i++) {
      frc846::math::VectorND<units::inch, 2> location{
          kModuleLocationSigns[i].first * constants_.wheelbase_horizontal_dim,
          kModuleLocationSigns[i].second * constants_.wheelbase_forward_dim};

      units::degree_t rot_direction = location.angle(false) - 90_deg;

      frc846::math::VectorND<units::feet_per_second, 2> rotation_vector{
          rotation * units::math::cos(rot_direction) * radius / 1_rad,
          rotation * units::math::sin(rot_direction) * radius / 1_rad};

      module_targets[i] =
          inputs.translation_target.rotate(-inputs.bearing, true) +
          rotation_vector;
    }
  };

  auto get_max_mag = [&]() {
    units::feet_per_second_t max_mag = 0_fps;
    for (int i = 0; i < 4; i++) {
      if (module_targets[i].magnitude() > max_mag) {
        max_mag = module_targets[i].magnitude();
      }
    }
    return max_mag;
  };

  auto rescale_outputs = [&](units::feet_per_second_t max_mag) {
    for (int i = 0; i < 4; i++) {
      module_targets[i] *= inputs.max_speed / max_mag;
    }
  };

  calculate_module_targets(inputs.rotation_target);

  if (inputs.cut_excess_steering) {
    units::radians_per_second_t rotation_target = inputs.rotation_target;
    units::feet_per_second_t max_mag;

    do {
      calculate_module_targets(rotation_target);

      max_mag = get_max_mag();

      if (max_mag > inputs.max_speed) {
        rotation_target -= constants_.rotation_iter_dec;
        if (rotation_target < 0_rad_per_s) {
          calculate_module_targets(0_rad_per_s);
          max_mag = get_max_mag();
          if (max_mag > inputs.max_speed) { rescale_outputs(get_max_mag()); }
          break;
        }
      } else {
        break;
      }
    } while (max_mag > inputs.max_speed);
  } else {
    units::feet_per_second_t max_mag = get_max_mag();
    if (max_mag > inputs.max_speed) { rescale_outputs(max_mag); }
  }

  SwerveOpenLoopCalculatorOutput output{};
  for (int i = 0; i < 4; i++) {
    output.drive_outputs[i] = module_targets[i].magnitude();
    output.steer_outputs[i] = module_targets[i].angle(true);
  }
  return output;
}

}  // namespace frc846::robot::swerve::control