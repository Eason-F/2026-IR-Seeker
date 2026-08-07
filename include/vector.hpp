#pragma once

#include <cmath>

namespace ir {

// A 2D vector in robot coordinates:
//   +x is right, +y is forward, and positive rotation is clockwise.
class Vector2 {
 public:
  constexpr Vector2() : x_(0.0F), y_(0.0F) {}
  constexpr Vector2(float x, float y) : x_(x), y_(y) {}

  static Vector2 fromBearingDegrees(float bearingDegrees,
                                    float magnitude = 1.0F) {
    const float radians = bearingDegrees * degreesToRadians();
    return Vector2(std::sin(radians) * magnitude,
                   std::cos(radians) * magnitude);
  }

  constexpr float x() const { return x_; }
  constexpr float y() const { return y_; }

  constexpr float magnitudeSquared() const { return x_ * x_ + y_ * y_; }
  float magnitude() const { return std::sqrt(magnitudeSquared()); }

  bool isNearZero(float epsilon = 1.0e-6F) const {
    return magnitudeSquared() <= epsilon * epsilon;
  }

  // A zero-length vector has no direction, so it normalizes to (0, 0).
  Vector2 normalized(float epsilon = 1.0e-6F) const {
    const float length = magnitude();
    return length <= epsilon ? Vector2() : *this / length;
  }

  // Returns a clockwise bearing in the range (-180, 180]. A zero vector
  // returns 0 degrees.
  float bearingDegrees() const {
    if (isNearZero()) return 0.0F;

    float bearing = std::atan2(x_, y_) * radiansToDegrees();
    if (bearing <= -180.0F) bearing += 360.0F;
    return bearing;
  }

  Vector2 rotatedDegrees(float clockwiseDegrees) const {
    const float radians = clockwiseDegrees * degreesToRadians();
    const float sine = std::sin(radians);
    const float cosine = std::cos(radians);
    return Vector2(x_ * cosine + y_ * sine,
                   y_ * cosine - x_ * sine);
  }

  constexpr float dot(const Vector2 &other) const {
    return x_ * other.x_ + y_ * other.y_;
  }

  constexpr Vector2 operator+(const Vector2 &other) const {
    return Vector2(x_ + other.x_, y_ + other.y_);
  }

  constexpr Vector2 operator-(const Vector2 &other) const {
    return Vector2(x_ - other.x_, y_ - other.y_);
  }

  constexpr Vector2 operator-() const { return Vector2(-x_, -y_); }

  constexpr Vector2 operator*(float scalar) const {
    return Vector2(x_ * scalar, y_ * scalar);
  }

  constexpr Vector2 operator/(float scalar) const {
    return Vector2(x_ / scalar, y_ / scalar);
  }

  Vector2 &operator+=(const Vector2 &other) {
    x_ += other.x_;
    y_ += other.y_;
    return *this;
  }

  Vector2 &operator-=(const Vector2 &other) {
    x_ -= other.x_;
    y_ -= other.y_;
    return *this;
  }

  Vector2 &operator*=(float scalar) {
    x_ *= scalar;
    y_ *= scalar;
    return *this;
  }

  Vector2 &operator/=(float scalar) {
    x_ /= scalar;
    y_ /= scalar;
    return *this;
  }

 private:
  static constexpr float degreesToRadians() {
    return 0.01745329251994329577F;
  }

  static constexpr float radiansToDegrees() {
    return 57.295779513082320876F;
  }

  float x_;
  float y_;
};

constexpr Vector2 operator*(float scalar, const Vector2 &vector) {
  return vector * scalar;
}

}  // namespace ir
