#include <cmath>
#include <memory>
#include "accel.h"
#include "constant.h"
#include "exception.h"
#include "geometry.h"
#include "stat.h"

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
  const int SEARCH_DIV_RES = 2;
  const int MIN_OBJS = 20;
  Aabb a = boundingBox(fs);
  n->p = a.p;
  n->m = a.m;
  if (fs.size() > MIN_OBJS) {
    std::vector<Face> fs1 = fs, fs2;
    double min_c = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < SEARCH_DIV_RES; j++) {
        Vector3d tmp_p = n->p;
        tmp_p.p[i] = n->m[i] + (n->p[i] - n->m[i]) / SEARCH_DIV_RES * j;

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
  static NodeStore ns;

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
