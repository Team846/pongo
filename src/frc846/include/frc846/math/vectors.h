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
template <typename T, size_t N> class VectorND {
  static_assert(
      N > 0, "VectorND can not be created with less than one dimension.");
  static_assert(units::traits::is_unit_t<T>(),
      "VectorND can only be created with unit types.");

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
  VectorND(const VectorND<T, N>& other) : VectorND() {
    for (size_t i = 0; i < N; ++i) {
      data[i] = other[i];
    }
  }

  // Adds VectorND<T, N> to this and returns result
  VectorND<T, N> operator+(const VectorND<T, N>& other) const {
    VectorND<T, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] + other[i];
    }
    return result;
  }

  // Subtracts VectorND<T, N> from this and returns result
  VectorND<T, N> operator-(const VectorND<T, N>& other) const {
    VectorND<T, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] - other[i];
    }
    return result;
  }

  VectorND<T, N> operator*(const double scalar) const {
    VectorND<T, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] * scalar;
    }
    return result;
  }

  friend double operator*(double lhs, const VectorND<T, N>& rhs) {
    return rhs * lhs;
  }

  VectorND<T, N> operator/(const double scalar) const {
    VectorND<T, N> result;
    for (size_t i = 0; i < N; ++i) {
      result[i] = data[i] / scalar;
    }
    return result;
  }

  VectorND<T, N>& operator+=(const VectorND<T, N>& other) {
    for (size_t i = 0; i < N; ++i) {
      data[i] += other[i];
    }
    return *this;
  }

  VectorND<T, N>& operator-=(const VectorND<T, N>& other) {
    for (size_t i = 0; i < N; ++i) {
      data[i] -= other[i];
    }
    return *this;
  }

  VectorND<T, N>& operator*=(const double scalar) {
    for (size_t i = 0; i < N; ++i) {
      data[i] *= scalar;
    }
    return *this;
  }

  VectorND<T, N>& operator/=(const double scalar) {
    for (size_t i = 0; i < N; ++i) {
      data[i] /= scalar;
    }
    return *this;
  }

  void operator=(const VectorND<T, N>& other) {
    for (size_t i = 0; i < N; ++i) {
      data[i] = other[i];
    }
  }

  // Uses 'safe' double comparison
  bool operator==(const VectorND<T, N>& other) const {
    for (size_t i = 0; i < N; ++i) {
      if (!frc846::math::DEquals(data[i], other[i])) { return false; }
    }
    return true;
  }

  // Returns a vector rotated by a given angle. Default is clockwise rotation.
  VectorND<T, N> rotate(units::degree_t angle, bool clockwise = true) const {
    static_assert(N == 2, "Rotation is only defined for 2D vectors.");
    if (clockwise) { angle = -angle; }
    return {
        data[0] * units::math::cos(angle) - data[1] * units::math::sin(angle),
        data[0] * units::math::sin(angle) + data[1] * units::math::cos(angle)};
  }

  // Returns the dot product of this vector and another
  T dot(const VectorND<T, N>& other) const {
    T result = T{};
    for (size_t i = 0; i < N; ++i) {
      result += data[i] * other[i].template to<double>();
    }
    return result;
  }

  // Returns the cross product of this vector and another
  // Cross product is only defined for 3D vectors
  VectorND<T, N> cross(const VectorND<T, N>& other) const {
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
  VectorND<T, N> unit() const {
    return *this / magnitude().template to<double>();
  }

  // Projects this vector onto another and returns
  VectorND<T, N> projectOntoAnother(const VectorND<T, N>& other) const {
    return other.projectOntoThis(*this);
  }

  // Projects another vector onto this and returns
  VectorND<T, N> projectOntoThis(const VectorND<T, N>& other) const {
    return unit() * dot(other).template to<double>();
  }

  // Returns the angle of this vector
  // If angleIsBearing is true, 0_deg is +y and angles are measured clockwise
  // Otherwise, 0_deg is +x and angles are measured counter-clockwise
  units::degree_t angle(bool angleIsBearing = false) const {
    assert(N == 2 && "Angle can only be calculated for 2D vectors.");
    if (angleIsBearing) { return units::math::atan2(data[0], data[1]); }
    return units::math::atan2(data[1], data[0]);
  }

  // Returns the angle between this vector and another
  units::degree_t angleTo(
      const VectorND<T, N>& other, bool angleIsBearing = false) const {
    return other.angle(angleIsBearing) - angle(angleIsBearing);
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
using Vector1D = VectorND<units::inch_t, 1>;
// 2D vector, units::inch_t
using Vector2D = VectorND<units::inch_t, 2>;
// 3D vector, units::inch_t
using Vector3D = VectorND<units::inch_t, 3>;

}  // namespace frc846::math