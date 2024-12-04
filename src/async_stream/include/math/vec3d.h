#pragma once

#include <cmath>
#include <string>

// for kMathEpsilon
#include "math/vec2d.h"

/**
 * @namespace keti::common::math
 * @brief math
 */
namespace keti {
namespace common {
namespace math {

/**
 * @class Vec3d
 *
 * @brief Implements a class of 2-dimensional vectors.
 */
class Vec3d {
 public:
  //! Constructor which takes x- and y-coordinates.
  constexpr Vec3d(const double x, const double y, const double z) noexcept : x_(x), y_(y), z_(z) {}

  //! Constructor returning the zero vector.
  constexpr Vec3d() noexcept : Vec3d(0, 0, 0) {}

  //! Creates a unit-vector with the roll pitch and yaw angles about fixed axes X, Y, Z respectively
  static Vec3d CreateUnitVec3d(const double roll,
                               const double pitch,
                               const double yaw);

  //! Getter for x component
  double x() const { return x_; }

  //! Getter for y component
  double y() const { return y_; }

  //! Getter for z component
  double z() const { return z_; }

  //! Setter for x component
  void set_x(const double x) { x_ = x; }

  //! Setter for y component
  void set_y(const double y) { y_ = y; }

  //! Setter for y component
  void set_z(const double z) { z_ = z; }

  //! Gets the length of the vector
  double Length() const;

  //! Gets the squared length of the vector
  double LengthSquare() const;

  //! Gets the angle between the vector and the positive x semi-axis
  double GetYaw();

  //! Returns the unit vector that is co-linear with this vector
  void Normalize();

  //! Returns the distance to the given vector
  double DistanceTo(const Vec3d &other) const;

  //! Returns the squared distance to the given vector
  double DistanceSquareTo(const Vec3d &other) const;

  //! Returns the "cross" product between these two Vec3d (non-standard)
  Vec3d CrossProd(const Vec3d &other) const;

  //! Returns the inner product between these two Vec3d
  double InnerProd(const Vec3d &other) const;

  //! Returns the rotation matrix by X-Y-Z angles
  static double** RotateMatZYX(const double x_angle,
                              const double y_angle,
                              const double z_angle);

  //! Retruns the quaternion from euler angles
  static double* ToQuaternion(const double roll, const double pitch, const double yaw);

  //! Retruns the euler angles from quaternion
  static double* ToEulerAngles(double* q);  

  //! rotate the vector by X-Y-Z angles
  Vec3d rotate(const double roll, const double pitch, const double yaw) const;

  //! rotate the vector itself by X-Y-Z angles
  void SelfRotate(const double roll, const double pitch, const double yaw);

  //! Sums two Vec3d
  Vec3d operator+(const Vec3d &other) const;

  //! Subtracts two Vec3d
  Vec3d operator-(const Vec3d &other) const;

  //! Multiplies Vec3d by a scalar
  Vec3d operator*(const double ratio) const;

  //! Divides Vec3d by a scalar
  Vec3d operator/(const double ratio) const;

  //! Sums another Vec3d to the current one
  Vec3d &operator+=(const Vec3d &other);

  //! Subtracts another Vec3d to the current one
  Vec3d &operator-=(const Vec3d &other);

  //! Multiplies this Vec3d by a scalar
  Vec3d &operator*=(const double ratio);

  //! Divides this Vec3d by a scalar
  Vec3d &operator/=(double ratio);

  //! Compares two Vec3d
  bool operator==(const Vec3d &other) const;

  //! Returns a human-readable string representing this object
  std::string DebugString() const;

 protected:
  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.0;
};

//! Multiplies the given Vec3d by a given scalar
Vec3d operator*(const double ratio, const Vec3d &vec);

}  // namespace common
}  // namespace math
}  // namespace keti