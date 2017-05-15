#ifndef ACCEL_H_
#define ACCEL_H_

#include "geometry.h"
#include <stack>

class Aabb {
 public:
  Vector3d p, m;

  Aabb(Vector3d p_, Vector3d m_) : p(p_), m(m_) {}
  Aabb() {}
  double volume() {
    return (p.x() - m.x()) * (p.y() - m.y()) * (p.z() - m.z());
  }
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

std::unique_ptr<AccelNode> separateGeometryBvh(std::vector<Face> fs);
#endif
