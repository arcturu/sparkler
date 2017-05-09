#ifndef ACCEL_H_
#define ACCEL_H_

#include "geometry.h"

class Aabb {
 public:
  Vector3d p, m;

  Aabb(Vector3d p_, Vector3d m_) : p(p_), m(m_) {}
  double volume() {
    return (p.x - m.x) * (p.y - m.y) * (p.z - m.z);
  }
};

class AccelNode {
 public:
  std::vector<std::unique_ptr<AccelNode>> children;
  std::vector<Face> faces;
  Vector3d p, m; // p: +xyz, n: -xyz boundery

  void dump();
  Intersection traverse(const Ray& ray);
};

class AccelStructure {
 public:
  std::shared_ptr<Geometry> geo;

  AccelStructure(std::shared_ptr<Geometry> geo_) : geo(geo_) {}
  virtual Intersection intersect(const Ray& ray) = 0;
};

class AccelNaive : public AccelStructure {
 public:
  AccelNaive(std::shared_ptr<Geometry> geo_) : AccelStructure(geo_) {}
  Intersection intersect(const Ray& ray);
};

class AccelBvh : public AccelStructure {
 public:
  std::unique_ptr<AccelNode> root;

  AccelBvh(std::shared_ptr<Geometry> geo_);
  Intersection intersect(const Ray& ray);
};

#endif
