#pragma once

#include <units/angle.h>
#include <units/constants.h>
#include <units/length.h>
#include <units/math.h>

#include <cassert>
#include <cmath>
#include <vector>

#include "frc846/math/collection.h"

namespace frc846::math {
template <typename UT, size_t N> class VectorND {
  static_assert(
      N > 0, "VectorND can not be created with less than one dimension.");
  static_assert(units::traits::is_unit<UT>(),
      "VectorND can only be created with unit types. Unit_t is invalid.");

  using T = units::unit_t<UT>;

private:
  std::vector<T> data;

public:
  // Default constructor initializes with zeros
  VectorND() : data{N, T()} {}

  // Constructs an N-dimensional vector from N unit-type values
  VectorND(std::initializer_list<T> dims) : VectorND() {
    assert(
        dims.size() == N && "VectorND must be constructed with N dimensions.");
    std::copy(dims.begin(), dims.end(), data.begin());
  }

  // Constructs a 2D vector from magnitude and angle
  // If angleIsBearing is true, 0_deg is +y and angles are measured clockwise
  // Otherwise, 0_deg is +x and angles are measured counter-clockwise
  VectorND(T magnitude, units::degree_t theta, bool angleIsBearing = false)
      : VectorND() {
    assert(N == 2 && "Polar constructor can only be used with 2D vectors.");
    if (angleIsBearing) { theta = 90_deg - theta; }
    data[0] = magnitude * units::math::cos(theta);
    data[1] = magnitude * units::math::sin(theta);
  }

  // Constructs 2D vector from a pair
  VectorND(const std::pair<T, T> vec) : VectorND() {
    assert(N == 2 && "Pair constructor can only be used with 2D vectors.");
    data = {vec.first, vec.second};
  }

  // Copy constructor for VectorND
  VectorND(const VectorND<UT, N>& other) : VectorND() {
    for (size_t i = 0; i < N; ++i) {
      data[i] = other[i];
    }
  }

  // Adds VectorND<UT, N> to this and returns result
  VectorND<UT, N> operator+(const VectorND<UT, N>& other) const {
    VectorND<UT, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] + other[i];
    }
    return result;
  }

  // Subtracts VectorND<UT, N> from this and returns result
  VectorND<UT, N> operator-(const VectorND<UT, N>& other) const {
    VectorND<UT, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] - other[i];
    }
    return result;
  }

  VectorND<UT, N> operator*(const double scalar) const {
    VectorND<UT, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] * scalar;
    }
    return result;
  }

  friend double operator*(double lhs, const VectorND<UT, N>& rhs) {
    return rhs * lhs;
  }

  VectorND<UT, N> operator/(const double scalar) const {
    VectorND<UT, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] / scalar;
    }
    return result;
  }

  VectorND<UT, N>& operator+=(const VectorND<UT, N>& other) {
    for (size_t i = 0; i < N; ++i) {
      data[i] += other[i];
    }
    return *this;
  }

  VectorND<UT, N>& operator-=(const VectorND<UT, N>& other) {
    for (size_t i = 0; i < N; ++i) {
      data[i] -= other[i];
    }
    return *this;
  }

  VectorND<UT, N>& operator*=(const double scalar) {
    for (size_t i = 0; i < N; ++i) {
      data[i] *= scalar;
    }
    return *this;
  }

  VectorND<UT, N>& operator/=(const double scalar) {
    for (size_t i = 0; i < N; ++i) {
      data[i] /= scalar;
    }
    return *this;
  }

  void operator=(const VectorND<UT, N>& other) {
    for (size_t i = 0; i < N; ++i) {
      data[i] = other[i];
    }
  }

  // Uses 'safe' double comparison
  bool operator==(const VectorND<UT, N>& other) const {
    for (size_t i = 0; i < N; ++i) {
      if (!frc846::math::DEquals(data[i], other[i])) { return false; }
    }
    return true;
  }

  // Returns a vector rotated by a given angle. Default is clockwise rotation.
  VectorND<UT, N> rotate(units::degree_t angle, bool clockwise = true) const {
    static_assert(N == 2, "Rotation is only defined for 2D vectors.");
    if (clockwise) { angle = -angle; }
    return {
        data[0] * units::math::cos(angle) - data[1] * units::math::sin(angle),
        data[0] * units::math::sin(angle) + data[1] * units::math::cos(angle)};
  }

  // Returns the dot product of this vector and another with the units of the
  // other vector
  template <typename UT2>
  units::unit_t<UT2> dot(const VectorND<UT2, N>& other) const {
    units::unit_t<UT2> result{};
    for (size_t i = 0; i < N; ++i) {
      result += data[i].template to<double>() * other[i];
    }
    return result;
  }

  // Returns the cross product of this vector and another
  // Cross product is only defined for 3D vectors
  template <typename UT2>
  VectorND<units::compound_unit<UT, UT2>, N> cross(
      const VectorND<UT2, N>& other) const {
    static_assert(N == 3, "Cross product is only defined for 3D vectors.");
    return {data[1] * other[2] - data[2] * other[1],
        data[2] * other[0] - data[0] * other[2],
        data[0] * other[1] - data[1] * other[0]};
  }

  // Returns the magnitude of this vector
  T magnitude() const {
    auto result = data[0] * data[0];
    for (size_t i = 1; i < N; ++i) {
      result += data[i] * data[i];
    }
    return units::math::sqrt(result);
  }

  // Returns the unit vector of this vector
  VectorND<UT, N> unit() const {
    return *this / magnitude().template to<double>();
  }

  // Projects this vector onto another and returns
  template <typename UT2>
  VectorND<UT, N> projectOntoAnother(const VectorND<UT2, N>& other) const {
    return other.projectOntoThis(*this);
  }

  // Projects another vector onto this and returns
  template <typename UT2>
  VectorND<UT2, N> projectOntoThis(const VectorND<UT2, N>& other) const {
    assert(N == 2 && "Projection is only defined for 2D vectors.");
    double unit_x = unit()[0].template to<double>();
    double unit_y = unit()[1].template to<double>();
    return {unit_x * dot(other), unit_y * dot(other)};
  }

  // Returns the angle of this vector
  // If angleIsBearing is true, 0_deg is +y and angles are measured clockwise
  // Otherwise, 0_deg is +x and angles are measured counter-clockwise
  units::degree_t angle(bool angleIsBearing = false) const {
    assert(N == 2 && "Angle can only be calculated for 2D vectors.");
    if (angleIsBearing) { return units::math::atan2(data[0], data[1]); }
    try {
      return units::math::atan2(data[1], data[0]);
    } catch (std::exception& exc) {
      (void)exc;
      return 0_deg;
    }
  }

  // Returns the angle between this vector and another
  template <typename UT2>
  units::degree_t angleTo(
      const VectorND<UT2, N>& other, bool angleIsBearing = false) const {
    return other.angle(angleIsBearing) - angle(angleIsBearing);
  }

  // Returns a modified vector with a given delta added to its magnitude
  VectorND<UT, N> AddToMagnitude(T delta) {
    assert(N == 2 && "AddToMagnitude is only defined for 2D vectors.");
    return {magnitude() + delta, angle(true), true};
  }

  // Returns a modified vector to the resized magniutde
  VectorND<UT, N> resize(T magnitude) {
    assert(N == 2 && "resize is only defined for 2D vectors.");
    return {magnitude, angle(true), true};
  }

  // Const and non-const accessors for vector elements
  const T& operator[](size_t i) const { return data[i]; }
  T& operator[](size_t i) { return data[i]; }

  // Returns a pair representation of a 2D vector
  std::pair<T, T> toPair() {
    static_assert(N == 2 && "toPair can only be used with 2D vectors.");
    return {data[0], data[1]};
  }

  // Returns string representation of this vector
  std::string toString() const {
    std::string output = "<";
    for (size_t i = 0; i < N; ++i) {
      output += data[i];
      if (i < N - 1) output += ", ";
    }
    output += ">";
    return output;
  }
};

// Commonly used vector types

// 1D vector, units::inch_t
using Vector1D = VectorND<units::inch, 1>;
// 2D vector, units::inch_t
using Vector2D = VectorND<units::inch, 2>;
// 3D vector, units::inch_t
using Vector3D = VectorND<units::inch, 3>;

struct Line {
  frc846::math::Vector2D point;
  units::degree_t angle;

  frc846::math::Vector2D intersect(Line other) {
    units::inch_t x = (point[1] - other.point[1] +
                          units::math::tan(other.angle) * other.point[0] -
                          units::math::tan(angle) * point[0]) /
                      (units::math::tan(other.angle) - units::math::tan(angle));
    units::inch_t y = units::math::tan(angle) * (x - point[0]) + point[1];
    return {x, y};
  }

  void translate(frc846::math::Vector2D addend) { point += addend; }
};

}  // namespace frc846::math