#include <cmath>
#include <memory>
#include <limits>
#include <queue>
#include "accel.h"
#include "constant.h"
#include "exception.h"
#include "geometry.h"
#include "stat.h"
#include "global.h"

#include <iostream>

/*
Intersection AccelNaive::intersect(const Ray& ray) {
  Intersection it;
  double min_t = std::numeric_limits<double>::infinity();

  for (auto f = geo->fs.begin(); f != geo->fs.end(); ++f) {
    Intersection ittmp = Geometry::intersectTriangle(*f, ray);
    if (ittmp.hit && ittmp.t < min_t) {
      min_t = ittmp.t;
      it = ittmp;
    }
  }

  return it;
}
*/

bool belongs(Vector3d v, Vector3d m, Vector3d p) {
  return v.x() >= m.x() && v.x() <= p.x() &&
         v.y() >= m.y() && v.y() <= p.y() &&
         v.z() >= m.z() && v.z() <= p.z();
}

Aabb boundingBox(const std::vector<Face>& fs) {
  const double inf = std::numeric_limits<double>::infinity();
  Vector3d p(-inf, -inf, -inf), m(inf, inf, inf);
  for (auto f = fs.begin(); f != fs.end(); ++f) {
    for (auto v = f->vs.begin(); v != f->vs.end(); ++v) {
      if (p.x() < v->p->x()) { p.x(v->p->x()); }
      if (p.y() < v->p->y()) { p.y(v->p->y()); }
      if (p.z() < v->p->z()) { p.z(v->p->z()); }
      if (m.x() > v->p->x()) { m.x(v->p->x()); }
      if (m.y() > v->p->y()) { m.y(v->p->y()); }
      if (m.z() > v->p->z()) { m.z(v->p->z()); }
    }
  }
  Aabb a(p, m);
  return a;
}

Aabb boundingBox(const std::vector<std::unique_ptr<Object>>& objects) {
  const double inf = std::numeric_limits<double>::infinity();
  Vector3d p(-inf, -inf, -inf), m(inf, inf, inf);
  Aabb box;
  for (const auto& obj : objects) {
    box.merge(obj->getAabb());
  }
  return box;
}

double costSeparationBvh(const std::vector<Face>& fs1, const std::vector<Face>& fs2, double sv) {
  Aabb a1 = boundingBox(fs1);
  Aabb a2 = boundingBox(fs2);
  double a1v = a1.volume();
  double a2v = a2.volume();
  return a1v / sv * fs1.size() + a2v / sv * fs2.size();
}

std::unique_ptr<AccelNode> separateGeometryBvh(std::vector<Face> fs) {
  stat_num_accel_node++;
  std::unique_ptr<AccelNode> n(new AccelNode);
  Aabb a = boundingBox(fs);
  n->p = a.p;
  n->m = a.m;
  if (fs.size() > G_MIN_OBJS) {
    std::vector<Face> fs1 = fs, fs2;
    double min_c = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < G_SEARCH_DIV_RES; j++) {
        Vector3d tmp_p = n->p;
        tmp_p.p[i] = n->m[i] + (n->p[i] - n->m[i]) / G_SEARCH_DIV_RES * j;

        std::vector<Face> tmp_fs1, tmp_fs2;
        for (auto f = fs.begin(); f != fs.end(); ++f) {
          bool in1 = false;
          for (auto v = f->vs.begin(); v != f->vs.end(); ++v) {
            if (belongs(*v->p, n->m, tmp_p)) {
              in1 = true;
              break;
            }
          }
          if (in1) {
            tmp_fs1.push_back(*f);
          } else {
            tmp_fs2.push_back(*f);
          }
        }

        double c = costSeparationBvh(tmp_fs1, tmp_fs2, a.volume());
        if (c < min_c) {
          min_c = c;
          fs1 = tmp_fs1;
          fs2 = tmp_fs2;
        }
      }
    }
    if (fs1.size() != 0 && fs2.size() != 0 && (fs1.size() < fs.size() || fs2.size() < fs.size())) {
      n->children.push_back(separateGeometryBvh(fs1));
      n->children.push_back(separateGeometryBvh(fs2));
    } else {
      stat_num_accel_leaf++;
      n->faces = fs;
    }
  } else {
    stat_num_accel_leaf++;
    n->faces = fs;
  }
  return n;
}

bool intersectBox(const Vector3d& m, const Vector3d& p, const Ray& ray, double *t_hit = nullptr) {
  stat_num_intersectBox++;
  double t_max = std::numeric_limits<double>::infinity();
  double t_min = -std::numeric_limits<double>::infinity();
  double t1, t2, t_far, t_near;

  *t_hit = t_max;

  for (int i = 0; i < 3; i++) {
    t1 = (m.p[i] - ray.src.p[i]) / ray.dir.p[i];
    t2 = (p.p[i] - ray.src.p[i]) / ray.dir.p[i];
    t_far = std::max(t1, t2);
    t_near = std::min(t1, t2);
    t_max = std::min(t_max, t_far);
    t_min = std::max(t_min, t_near);

    if (t_max < t_min) {
      return false;
    }
  }

  *t_hit = t_min;
  return true;
}

Intersection AccelNode::traverseLoop(const Ray& ray) {
  NodeStore ns;

  Intersection it;
  double min_t = std::numeric_limits<double>::infinity();
  double t;
  int stat_max_num_ns = 0;
  ns.push(this);
  while (!ns.empty()) {
    stat_num_traverse++;
    // TODO: improve search order
    AccelNode *n = ns.top(); ns.pop();
    if (n->children.size() == 0) { // n is a leef node
      for (int i = 0; i < n->faces.size(); i++) {
        Intersection tmp_it = Geometry::intersectTriangle(n->faces[i], ray);
        if (tmp_it.hit && tmp_it.t >= 0 && tmp_it.t < min_t) {
          min_t = tmp_it.t;
          it = tmp_it;
        }
      }
    } else {
      for (int i = 0; i < n->children.size(); i++) {
        bool hit = intersectBox(n->children[i]->m, n->children[i]->p, ray, &t);
        if (hit && t < min_t) {
          ns.push(n->children[i].get());
        }
      }
    }
    if (stat_max_num_ns < ns.size()) {
      stat_max_num_ns = ns.size();
    }
  }
//  if (stat_max_num_ns > 10) {
//    std::cout << "[stat] max_num_ns: " << stat_max_num_ns << std::endl;
//  }
  return it;
}

Intersection AccelNode::traverse(const Ray& ray, double min_t) {
  stat_num_traverse++;
  Intersection it;
  if (children.size() == 0) { // it's a leef
    for (auto f = faces.begin(); f != faces.end(); ++f) {
      Intersection ittmp = Geometry::intersectTriangle(*f, ray);
      if (ittmp.hit && ittmp.t < min_t) {
        min_t = ittmp.t;
        it = ittmp;
      }
    }
  } else {
    double t;
    for (int i = 0; i < children.size(); i++) {
      bool hit = intersectBox(children[i]->m, children[i]->p, ray, &t);
      if (hit && t < min_t) {
        Intersection ittmp = children[i]->traverse(ray, min_t);
        if (ittmp.hit && ittmp.t < min_t) {
          min_t = ittmp.t;
          it = ittmp;
        }
      }
    }
  }

  return it;
}

void AccelNode::dump() {
  if (children.size() == 0) {
    std::cout << "#faces: " << faces.size() << std::endl;
  } else {
    for (auto c = children.begin(); c != children.end(); ++c) {
      (*c)->dump();
    }
  }
}

std::unique_ptr<BvhNode> constructBvh(std::vector<std::unique_ptr<Object>> objects) {
  stat_num_accel_node++;
  std::unique_ptr<BvhNode> n(new BvhNode);
  Aabb a = boundingBox(objects);
  n->p = a.p;
  n->m = a.m;
  double max_volume = 0;
  if (objects.size() > G_MIN_OBJS) {
    std::vector<int> objs1, objs2;
    double min_c = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < G_SEARCH_DIV_RES; j++) {
        Vector3d tmp_p = n->p;
        tmp_p.p[i] = n->m[i] + (n->p[i] - n->m[i]) / (G_SEARCH_DIV_RES + 1) * (j + 1);
        Aabb tmp_bound(tmp_p, n->m);
        std::vector<int> tmp_objs1, tmp_objs2;
        Aabb tmp_box1, tmp_box2;
        for (int k = 0; k < objects.size(); k++) {
          if (max_volume < objects[k]->getAabb().volume()) {
            max_volume = objects[k]->getAabb().volume();
          }
          if (objects[k]->belongsTo(tmp_bound)) {
            tmp_objs1.push_back(k);
            tmp_box1.merge(objects[k]->getAabb());
          } else {
            tmp_objs2.push_back(k);
            tmp_box2.merge(objects[k]->getAabb());
          }
        }
        double sv = tmp_box1.volume() + tmp_box2.volume();
        double c = tmp_box1.volume() / sv * tmp_objs1.size()
          + tmp_box2.volume() / sv * tmp_objs2.size();
        if (c < min_c) {
          min_c = c;
          objs1 = tmp_objs1;
          objs2 = tmp_objs2;
        }
      }
    }
    printf("max volume: %f\n", max_volume);
    if (objs1.size() > 1 &&
        objs2.size() > 1 &&
        (objs1.size() < objects.size() || objs2.size() < objects.size())) {
      printf("divide: %lu %lu\n", objs1.size(), objs2.size());
      std::vector<std::unique_ptr<Object>> c1, c2;
      for (auto i : objs1) {
        c1.push_back(std::move(objects[i]));
      }
      for (auto i : objs2) {
        c2.push_back(std::move(objects[i]));
      }
      n->children.push_back(constructBvh(std::move(c1)));
      n->children.push_back(constructBvh(std::move(c2)));
    } else {
      printf("leaf: %lu\n", objects.size());
      stat_num_accel_leaf++;
      n->objects = std::move(objects);
    }
  } else {
    printf("leaf: %lu\n", objects.size());
    stat_num_accel_leaf++;
    n->objects = std::move(objects);
  }
  return n;
}

Intersection BvhNode::traverse(const Ray& ray, double min_t) {
  stat_num_traverse++;
  Intersection it;
  if (children.size() == 0) { // it's a leef
    for (const auto& obj: objects) {
      Intersection tmp_it = obj->intersect(ray);
      if (tmp_it.hit && tmp_it.t < min_t) {
        min_t = tmp_it.t;
        it = tmp_it;
      }
    }
  } else {
    double t;
    for (const auto& c : children) {
      bool hit = intersectBox(c->m, c->p, ray, &t);
      if (hit && t < min_t) {
        Intersection tmp_it = c->traverse(ray, min_t);
        if (tmp_it.hit && tmp_it.t < min_t) {
          min_t = tmp_it.t;
          it = tmp_it;
        }
      }
    }
  }

  return it;
}

class CompareDistance {
 public:
  bool operator()(std::pair<double, int> lhs, std::pair<double, int> rhs) {
    return lhs.first > rhs.first; // ascending order
  }
};

Intersection BvhNode::traverseLoop(const Ray& ray) {
  Store<BvhNode *> ns;

  Intersection it;
  it.t = std::numeric_limits<double>::infinity();
  double t;
  int stat_max_num_ns = 0;
  ns.push(this);
  while (!ns.empty()) {
    stat_num_traverse++;
    // TODO: improve search order
    BvhNode *n = ns.top(); ns.pop();
    if (n->children.size() == 0) { // n is a leef node
      for (const auto& obj : n->objects) {
        Intersection tmp_it = obj->intersect(ray);
        if (tmp_it.hit && tmp_it.t < it.t) {
          it = tmp_it;
        }
      }
    } else {
      for (const auto& c : n->children) {
        bool hit = intersectBox(c->m, c->p, ray, &t);
        if (hit && t < it.t) {
          ns.push(c.get());
        }
      }
//      std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, CompareDistance> queue;
//      for (int i = 0; i < n->children.size(); i++) {
//        intersectBox(n->children[i]->m, n->children[i]->p, ray, &t);
//        queue.push(std::make_pair(t, i));
//      }
////      printf("---\n");
//      int cc = 0;
//      while (!queue.empty()) {
//        std::pair<double, int> t = queue.top(); queue.pop();
////        printf("%f %d\n", t.first, t.second);
//        if (t.first < std::numeric_limits<double>::infinity() && t.first < it.t) {
//          cc++;
//          ns.push(n->children[t.second].get());
//        }
//      }
    }
    if (stat_max_num_ns < ns.size()) {
      stat_max_num_ns = ns.size();
    }
  }
//  if (stat_max_num_ns > 10) {
//    std::cout << "[stat] max_num_ns: " << stat_max_num_ns << std::endl;
//  }
  return it;
}
