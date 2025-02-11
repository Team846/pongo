#include "calculators/AntiTippingCalculator.h"

#include <units/force.h>
#include <units/torque.h>

#include "frc846/math/constants.h"
#include "subsystems/robot_constants.h"

frc846::math::Vector3D AntiTippingCalculator::elev_cg_position_{
    robot_constants::elevator::pos_x, robot_constants::elevator::pos_y,
    robot_constants::base::height};
frc846::math::Vector3D AntiTippingCalculator::tele_cg_position_{
    robot_constants::telescope::pos_x, robot_constants::telescope::pos_y,
    robot_constants::base::height};

frc846::math::Differentiator AntiTippingCalculator::elev_df_vel{};
frc846::math::Differentiator AntiTippingCalculator::elev_df_acc{};
frc846::math::Smoother AntiTippingCalculator::elev_acc_smoother{0.4};

frc846::math::Differentiator AntiTippingCalculator::tele_df_vel{};
frc846::math::Differentiator AntiTippingCalculator::tele_df_acc{};
frc846::math::Smoother AntiTippingCalculator::tele_acc_smoother{0.4};

units::newton_t AntiTippingCalculator::tele_force{};
units::newton_t AntiTippingCalculator::elev_force{};

void AntiTippingCalculator::SetElevatorHeight(units::inch_t height) {
  if (height > robot_constants::elevator::min_height_off_base) {
    elev_cg_position_[2] = height / 2.0;
  } else {
    elev_cg_position_[2] =
        ((robot_constants::elevator::elevator_weight *
             robot_constants::elevator::min_height_off_base / 2.0) +
            (robot_constants::elevator::end_effector_weight * height)) /
        (robot_constants::elevator::total_weight);
  }

  elev_df_vel.Calculate(elev_cg_position_[2].to<double>());
  elev_df_acc.Calculate(elev_df_vel.GetRate());
  elev_acc_smoother.Calculate(elev_df_acc.GetRate());
  elev_force = robot_constants::elevator::total_weight *
               elev_acc_smoother.Get() * 1_in / (1_s * 1_s);
}

void AntiTippingCalculator::SetTelescopeHeight(units::inch_t height) {
  tele_cg_position_[2] =
      (robot_constants::telescope::telescope_weight * (height / 2.0) +
          robot_constants::telescope::end_effector_weight * height) /
      (robot_constants::telescope::total_weight);

  tele_df_vel.Calculate(tele_cg_position_[2].to<double>());
  tele_df_acc.Calculate(tele_df_vel.GetRate());
  tele_acc_smoother.Calculate(tele_df_acc.GetRate());
  tele_force = robot_constants::telescope::total_weight *
               tele_acc_smoother.Get() * 1_in / (1_s * 1_s);
}

frc846::math::Vector3D AntiTippingCalculator::CalculateRobotCG() {
  double total_weight_scalar = robot_constants::total_weight.to<double>();

  frc846::math::Vector3D weighted_sum =
      (elev_cg_position_ *
              robot_constants::elevator::total_weight.to<double>() +
          tele_cg_position_ *
              robot_constants::telescope::total_weight.to<double>() +
          frc846::math::Vector3D{0_in, 0_in, robot_constants::base::height} *
              robot_constants::base::weight.to<double>());

  return weighted_sum / total_weight_scalar;
}

frc846::math::VectorND<units::feet_per_second_squared, 2>
AntiTippingCalculator::LimitAcceleration(
    frc846::math::VectorND<units::feet_per_second, 2> vel_vec,
    units::degree_t bearing) {
  if (units::math::abs(vel_vec[0]) < 0.05_fps) vel_vec[0] = 0.05_fps;
  if (units::math::abs(vel_vec[1]) < 0.05_fps) vel_vec[1] = 0.05_fps;

  frc846::math::VectorND<units::feet_per_second, 2> v_neg_dir =
      vel_vec.rotate(180_deg, true).unit();

  frc846::math::Vector3D robot_cg = CalculateRobotCG();

  std::pair<double, double> kModuleLocationSigns[4] = {
      {+0.5, +0.5},  // FR
      {-0.5, +0.5},  // FL
      {-0.5, -0.5},  // BL
      {+0.5, -0.5},  // BR
  };

  frc846::math::Vector2D wheel_vecs[4];

  for (size_t i = 0; i < 4; i++) {
    wheel_vecs[i] =
        frc846::math::Vector2D{
            kModuleLocationSigns[i].first * robot_constants::base::wheelbase_x,
            kModuleLocationSigns[i].second * robot_constants::base::wheelbase_y}
            .rotate(bearing, true);
  }

  size_t closest_wheel_vec = 0U;
  units::degree_t closest_angle = 360_deg;

  for (size_t i = 0; i < 4; i++) {
    units::degree_t angle = units::math::abs(v_neg_dir.angleTo(wheel_vecs[i]));
    if (angle < closest_angle) {
      closest_angle = angle;
      closest_wheel_vec = i;
    }
  }

  frc846::math::Vector2D closest_wheel = wheel_vecs[closest_wheel_vec];

  frc846::math::Vector2D effective_wheel_vec =
      v_neg_dir.projectOntoThis<units::inch>(closest_wheel);
  frc846::math::Vector3D effective_wheel_vec_3d{
      effective_wheel_vec[0], effective_wheel_vec[1], 0_in};

  frc846::math::Vector3D r_vec = robot_cg - effective_wheel_vec_3d;

  units::feet_per_second_squared_t perm_accel_x =
      frc846::math::constants::physics::g * r_vec[0] / r_vec[2];
  units::feet_per_second_squared_t perm_accel_y =
      frc846::math::constants::physics::g * r_vec[1] / r_vec[2];

  frc846::math::Vector2D effective_tele_vec =
      v_neg_dir.projectOntoThis<units::inch>(
          closest_wheel -
          frc846::math::Vector2D{robot_constants::telescope::pos_x,
              robot_constants::telescope::pos_y});

  frc846::math::Vector2D effective_elev_vec =
      v_neg_dir.projectOntoThis<units::inch>(
          closest_wheel -
          frc846::math::Vector2D{robot_constants::elevator::pos_x,
              robot_constants::elevator::pos_y});

  perm_accel_x += (tele_force * effective_tele_vec[0] +
                      elev_force * effective_elev_vec[0]) /
                  (robot_constants::total_weight * r_vec[2]);

  perm_accel_y += (tele_force * effective_tele_vec[1] +
                      elev_force * effective_elev_vec[1]) /
                  (robot_constants::total_weight * r_vec[2]);

  return {perm_accel_x, perm_accel_y};
}