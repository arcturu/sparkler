#ifndef ACCEL_H_
#define ACCEL_H_

#include "geometry.h"

class AccelNode {
 public:
  std::vector<std::unique_ptr<AccelNode>> children;
  std::vector<Face> faces;
  Vector3d p, m; // p: +xyz, n: -xyz boundery

  void dump();
  Intersection traverse(Ray ray);
};

class AccelStructure {
 public:
  std::shared_ptr<Geometry> geo;

  AccelStructure(std::shared_ptr<Geometry> geo_) : geo(geo_) {}
  virtual Intersection intersect(Ray ray) = 0;
};

class AccelNaive : public AccelStructure {
 public:
  AccelNaive(std::shared_ptr<Geometry> geo_) : AccelStructure(geo_) {}
  Intersection intersect(Ray ray);
};

class AccelBvh : public AccelStructure {
 public:
  std::unique_ptr<AccelNode> root;

  AccelBvh(std::shared_ptr<Geometry> geo_);
  Intersection intersect(Ray ray);
};

#endif
