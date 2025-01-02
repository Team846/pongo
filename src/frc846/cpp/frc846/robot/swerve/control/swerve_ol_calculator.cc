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

  for (int i = 0; i < 4; i++) {
    frc846::math::VectorND<units::inch, 2> location{
        kModuleLocationSigns[i].first * constants_.wheelbase_horizontal_dim,
        kModuleLocationSigns[i].second * constants_.wheelbase_forward_dim};

    units::degree_t rot_direction = location.angle(false);

    frc846::math::VectorND<units::feet_per_second, 2> rotation{
        inputs.rotation_target * units::math::cos(rot_direction) * radius /
            1_rad,
        inputs.rotation_target * units::math::sin(rot_direction) * radius /
            1_rad};

    module_targets[i] = inputs.translation_target + rotation;
  }

  units::feet_per_second_t max_mag = 0_fps;
  for (int i = 0; i < 4; i++) {
    if (module_targets[i].magnitude() > max_mag) {
      max_mag = module_targets[i].magnitude();
    }
  }

  if (max_mag > inputs.max_speed) {
    for (int i = 0; i < 4; i++) {
      module_targets[i] *= inputs.max_speed / max_mag;
    }
  }

  SwerveOpenLoopCalculatorOutput output{};
  for (int i = 0; i < 4; i++) {
    output.drive_outputs[i] = module_targets[i].magnitude();
    output.steer_outputs[i] = module_targets[i].angle(true);
  }
  return output;
}

}  // namespace frc846::robot::swerve::control