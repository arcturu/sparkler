#ifndef VECTOR3D_H_
#define VECTOR3D_H_

#include <string>
#include "exception.h"

class Vector3d {
 public:
  double p[3];

  Vector3d() { p[0] = 0; p[1] = 0; p[2] = 0; }
  Vector3d(double x, double y, double z) { p[0] = x; p[1] = y; p[2] = z; }
  Vector3d cross(Vector3d v) const;
  double dot(Vector3d v) const;
  double length() const;
  Vector3d normalize() const;
  std::string toString() const;

  double x() const { return p[0]; }
  double y() const { return p[1]; }
  double z() const { return p[2]; }
  void x(double d) { p[0] = d; }
  void y(double d) { p[1] = d; }
  void z(double d) { p[2] = d; }
  double operator[](int i) const { return p[i]; };
  double& operator[](int i) { return p[i]; };
};

inline Vector3d operator-(const Vector3d v) {
  Vector3d r(-v.x(), -v.y(), -v.z());
  return r;
}

inline Vector3d operator-(const Vector3d l, const Vector3d r) {
  Vector3d v(l.x() - r.x(), l.y() - r.y(), l.z() - r.z());
  return v;
}

inline Vector3d operator+(const Vector3d l, const Vector3d r) {
  Vector3d v(l.x() + r.x(), l.y() + r.y(), l.z() + r.z());
  return v;
}

inline Vector3d operator*(double a, const Vector3d r) {
  Vector3d v(a * r.x(), a * r.y(), a * r.z());
  return v;
}

inline Vector3d operator/(const Vector3d l, double a) {
  Vector3d v(l.x() / a, l.y() / a, l.z() / a);
  return v;
}


#endif
