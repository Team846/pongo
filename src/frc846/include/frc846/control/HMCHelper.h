#pragma once

#include "HigherMotorController.h"

namespace frc846::control {

#define CHECK_HMC()                                      \
  if (hmc_ == nullptr)                                   \
    throw std::runtime_error("HMCHelper not bound to a " \
                             "HigherMotorController");

/*
HMCHelper

A class that provides a templated interface to HigherMotorController. Allows for
conversion from native units to system units.
*/
template <typename T> class HMCHelper {
public:
  using conv_unit =
      units::unit_t<units::compound_unit<T, units::inverse<units::turn>>>;
  using pos_unit = units::unit_t<T>;
  using vel_unit =
      units::unit_t<units::compound_unit<T, units::inverse<units::second>>>;

  HMCHelper() = default;

  void bind(HigherMotorController* hmc) { hmc_ = hmc; }

  void SetConversion(conv_unit conv) { conv_ = conv; }

  void WriteDC(double duty_cycle) {
    CHECK_HMC();
    hmc_->WriteDC(duty_cycle);
  }

  void WriteVelocity(vel_unit velocity) {
    CHECK_HMC();
    hmc_->WriteVelocity(velocity / conv_);
  }

  void WritePosition(pos_unit position) {
    CHECK_HMC();
    hmc_->WritePosition(position / conv_ - mark_offset);
  }

  void WriteVelocityOnController(vel_unit velocity) {
    CHECK_HMC();
    hmc_->WriteVelocityOnController(velocity / conv_);
  }

  void WritePositionOnController(pos_unit position) {
    CHECK_HMC();
    hmc_->WritePositionOnController(position / conv_);
  }

  vel_unit GetVelocity() {
    CHECK_HMC();
    return hmc_->GetVelocity() * conv_;
  }

  pos_unit GetPosition() {
    CHECK_HMC();
    return hmc_->GetPosition() * conv_ + mark_offset;
  }

  units::current::ampere_t GetCurrent() {
    CHECK_HMC();
    return hmc_->GetCurrent();
  }

  void SetPosition(pos_unit position) {
    CHECK_HMC();
    hmc_->SetPosition(position / conv_);
  }

  /*
  OffsetPositionTo()

  Does NOT zero motor encoder. Simply calculates then stores an offset, then
  adds it when GetPosition() is called. Subtracts offset when calling
  WritePosition().
  */
  void OffsetPositionTo(pos_unit position) {
    CHECK_HMC();
    mark_offset = position - GetPosition();
  }

  /*
  SetControllerSoftLimits()

  Set software limits for forward and reverse motion. Will be managed
  onboard the motor controller.
  */
  void SetControllerSoftLimits(pos_unit forward_limit, pos_unit reverse_limit) {
    CHECK_HMC();
    hmc_->SetControllerSoftLimits(forward_limit * conv_, reverse_limit * conv_);
  }

  /*
  SetSoftLimits()

  Set software limits for forward and reverse motion. Will be managed by
  HigherMotorController.
  */
  void SetSoftLimits(bool using_limits, pos_unit forward_limit = pos_unit(0),
      pos_unit reverse_limit = pos_unit(0),
      pos_unit forward_reduce = pos_unit(0),
      pos_unit reverse_reduce = pos_unit(0), double reduce_max_dc = 0.0) {
    hmc_->SetSoftLimits(config::SoftLimitsHelper<T>::CreateSoftLimits(conv_,
        using_limits, forward_limit, reverse_limit, forward_reduce,
        reverse_reduce, reduce_max_dc));
  }

private:
  HigherMotorController* hmc_;

  conv_unit conv_;

  pos_unit mark_offset{0};
};

}  // namespace frc846::control