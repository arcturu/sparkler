#ifndef ACCEL_H_
#define ACCEL_H_

#include "geometry.h"
#include <stack>

class Aabb {
 public:
  Vector3d p, m;

  Aabb(Vector3d p_, Vector3d m_) : p(p_), m(m_) {}
  double volume() {
    return (p.x() - m.x()) * (p.y() - m.y()) * (p.z() - m.z());
  }
};

class AccelNode {
 public:
  std::vector<std::unique_ptr<AccelNode>> children;
  std::vector<Face> faces;
  Vector3d p, m; // p: +xyz, n: -xyz boundery

  void dump();
  Intersection traverseLoop(const Ray& ray);
  Intersection traverse(const Ray& ray, double min_t);
};

template <class T>
class Store {
 public:
  std::vector<T> ns;

  Store() {
    ns.reserve(10);
  }
  void push(T n) {
    ns.push_back(n);
  }
  T top() {
    return ns[ns.size()-1];
  }
  void pop() {
    ns.pop_back();
  }
  bool empty() {
    return ns.empty();
  }
  int size() {
    return ns.size();
  }
};

// NOTE: RawStore is not so fast (and imperfect) comparing to Store
template <class T>
class RawStore {
 public:
  T ns[10]; // FIXME!!!
  int head;

  RawStore() {
    head = -1;
  }
  void push(T n) {
    ns[++head] = n;
  }
  T top() {
    return ns[head];
  }
  void pop() {
    head--;
  }
  bool empty() {
    return head < 0;
  }
  int size() {
    return head + 1;
  }
};

typedef Store<AccelNode *> NodeStore;

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
