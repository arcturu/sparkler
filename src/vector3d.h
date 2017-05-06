#ifndef VECTOR3D_H_
#define VECTOR3D_H_

#include <string>
#include "exception.h"

class Vector3d {
 public:
  double x, y, z;

  Vector3d() : x(0.0), y(0.0), z(0.0) {}
  Vector3d(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  Vector3d cross(Vector3d v) const;
  double dot(Vector3d v) const;
  double length() const;
  Vector3d normalize();
  std::string toString() const;
  double operator[](int i) const;
  double& operator[](int i);
};

Vector3d operator-(const Vector3d v);
Vector3d operator-(const Vector3d l, const Vector3d r);
Vector3d operator+(const Vector3d l, const Vector3d r);
Vector3d operator*(double a, const Vector3d r);
Vector3d operator/(const Vector3d l, double a);


#endif
