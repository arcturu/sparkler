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

inline Vector3d operator-(const Vector3d v);
inline Vector3d operator-(const Vector3d l, const Vector3d r);
inline Vector3d operator+(const Vector3d l, const Vector3d r);
inline Vector3d operator*(double a, const Vector3d r);
inline Vector3d operator/(const Vector3d l, double a);

inline Vector3d operator-(const Vector3d v) {
  Vector3d r(-v.x, -v.y, -v.z);
  return r;
}

inline Vector3d operator-(const Vector3d l, const Vector3d r) {
  Vector3d v(l.x - r.x, l.y - r.y, l.z - r.z);
  return v;
}

inline Vector3d operator+(const Vector3d l, const Vector3d r) {
  Vector3d v(l.x + r.x, l.y + r.y, l.z + r.z);
  return v;
}

inline Vector3d operator*(double a, const Vector3d r) {
  Vector3d v(a * r.x, a * r.y, a * r.z);
  return v;
}

inline Vector3d operator/(const Vector3d l, double a) {
  Vector3d v(l.x / a, l.y / a, l.z / a);
  return v;
}


#endif
