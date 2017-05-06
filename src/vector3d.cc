#include <cmath>
#include <sstream>
#include "vector3d.h"

Vector3d Vector3d::cross(Vector3d v) const {
  Vector3d c;
  c.x = y * v.z - z * v.y;
  c.y = z * v.x - x * v.z;
  c.z = x * v.y - y * v.x;
  return c;
}

double Vector3d::dot(Vector3d v) const {
  return x * v.x + y * v.y + z * v.z;
}

double Vector3d::length() const {
  return std::sqrt(x * x + y * y + z * z);
}

Vector3d Vector3d::normalize() {
  return *this / length();
}

std::string Vector3d::toString() const {
  std::stringstream sout;
  sout << "[" << x << "," << y << "," << z << "]";
  return sout.str();
}

double Vector3d::operator[](int i) const {
  switch (i) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      THROW_EXCEPTION("invalid argument");
  }
}

double& Vector3d::operator[](int i) {
  switch (i) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      THROW_EXCEPTION("invalid argument");
  }
}

Vector3d operator-(const Vector3d v) {
  Vector3d r(-v.x, -v.y, -v.z);
  return r;
}

Vector3d operator-(const Vector3d l, const Vector3d r) {
  Vector3d v(l.x - r.x, l.y - r.y, l.z - r.z);
  return v;
}

Vector3d operator+(const Vector3d l, const Vector3d r) {
  Vector3d v(l.x + r.x, l.y + r.y, l.z + r.z);
  return v;
}

Vector3d operator*(double a, const Vector3d r) {
  Vector3d v(a * r.x, a * r.y, a * r.z);
  return v;
}

Vector3d operator/(const Vector3d l, double a) {
  Vector3d v(l.x / a, l.y / a, l.z / a);
  return v;
}
