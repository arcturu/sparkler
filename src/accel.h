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
  Intersection traverse(const Ray& ray, double min_t);
};

class AccelStructure {
 public:
  static constexpr int HIST_COUNT = 10;
  std::shared_ptr<Geometry> geo;
  Face history[HIST_COUNT];
  int hist_head;
  int hist_tail;

  AccelStructure(std::shared_ptr<Geometry> geo_) : geo(geo_), hist_head(0), hist_tail(0) {}
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
