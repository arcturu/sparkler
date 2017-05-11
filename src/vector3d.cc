#include <cmath>
#include <sstream>
#include "vector3d.h"

Vector3d Vector3d::cross(Vector3d v) const {
  Vector3d c;
  c.x(y() * v.z() - z() * v.y());
  c.y(z() * v.x() - x() * v.z());
  c.z(x() * v.y() - y() * v.x());
  return c;
}

double Vector3d::dot(Vector3d v) const {
  return x() * v.x() + y() * v.y() + z() * v.z();
}

double Vector3d::length() const {
  return std::sqrt(x() * x() + y() * y() + z() * z());
}

Vector3d Vector3d::normalize() {
  return *this / length();
}

std::string Vector3d::toString() const {
  std::stringstream sout;
  sout << "[" << x() << "," << y() << "," << z() << "]";
  return sout.str();
}
