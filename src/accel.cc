#include <cmath>
#include <memory>
#include "accel.h"
#include "constant.h"
#include "exception.h"
#include "geometry.h"

#include <iostream>

Intersection AccelNaive::intersect(Ray ray) {
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

bool belongs(Vector3d v, Vector3d m, Vector3d p) {
  return v.x >= m.x && v.x <= p.x &&
         v.y >= m.y && v.y <= p.y &&
         v.z >= m.z && v.z <= p.z;
}

std::unique_ptr<AccelNode> separateGeometryBvh(std::vector<Face> fs) {
  std::unique_ptr<AccelNode> n(new AccelNode);
  const double inf = std::numeric_limits<double>::infinity();
  Vector3d p(-inf, -inf, -inf), m(inf, inf, inf);
  for (auto f = fs.begin(); f != fs.end(); ++f) {
    for (auto v = f->vs.begin(); v != f->vs.end(); ++v) {
      if (p.x < v->p->x) { p.x = v->p->x; }
      if (p.y < v->p->y) { p.y = v->p->y; }
      if (p.z < v->p->z) { p.z = v->p->z; }
      if (m.x > v->p->x) { m.x = v->p->x; }
      if (m.y > v->p->y) { m.y = v->p->y; }
      if (m.z > v->p->z) { m.z = v->p->z; }
    }
  }
  n->p = p;
  n->m = m;
  double wx = std::abs(p.x - m.x);
  double wy = std::abs(p.y - m.y);
  double wz = std::abs(p.z - m.z);
  if (fs.size() > 1 &&
      (wx > Constant::DIV_THRESHOLD ||
       wy > Constant::DIV_THRESHOLD ||
       wz > Constant::DIV_THRESHOLD)) {
    std::unique_ptr<AccelNode> c1(new AccelNode), c2(new AccelNode);
    if (wx > wy && wx > wz) {
      c1->m.x = m.x; c1->m.y = m.y; c1->m.z = m.z;
      c1->p.x = (m.x + p.x) / 2 - Constant::EPS; c1->p.y = p.y; c1->p.z = p.z;
      c2->m.x = (m.x + p.x) / 2 + Constant::EPS; c2->m.y = m.y; c2->m.z = m.z;
      c2->p.x = p.x; c2->p.y = p.y; c2->p.z = p.z;
    } else if (wy > wx && wy > wz) {
      c1->m.x = m.x; c1->m.y = m.y; c1->m.z = m.z;
      c1->p.x = p.x; c1->p.y = (m.y + p.y) / 2 + Constant::EPS; c1->p.z = p.z;
      c2->m.x = m.x; c2->m.y = (m.y + p.y) / 2 + Constant::EPS; c2->m.z = m.z;
      c2->p.x = p.x; c2->p.y = p.y; c2->p.z = p.z;
    } else { // wz is the largest or else
      c1->m.x = m.x; c1->m.y = m.y; c1->m.z = m.z;
      c1->p.x = p.x; c1->p.y = p.y; c1->p.z = (m.z + p.z) / 2 + Constant::EPS;
      c2->m.x = m.x; c2->m.y = m.y; c2->m.z = (m.z + p.z) / 2 + Constant::EPS;
      c2->p.x = p.x; c2->p.y = p.y; c2->p.z = p.z;
    }
    std::vector<Face> fs1, fs2;
    for (auto f = fs.begin(); f != fs.end(); ++f) {
      bool in1 = false;
      for (auto v = f->vs.begin(); v != f->vs.end(); ++v) {
        if (belongs(*v->p, c1->m, c1->p)) {
          in1 = true;
          break;
        }
      }
      if (in1) {
        fs1.push_back(*f);
      } else {
        fs2.push_back(*f);
      }
    }

    if (fs1.size() != 0 && fs2.size() != 0 && (fs1.size() < fs.size() || fs2.size() < fs.size())) {
      c1 = separateGeometryBvh(fs1);
      c2 = separateGeometryBvh(fs2);

      n->children.push_back(std::move(c1));
      n->children.push_back(std::move(c2));
    } else {
      n->faces = fs;
    }
  } else {
    n->faces = fs;
  }
  return n;
}

// prepare BVH
AccelBvh::AccelBvh(std::shared_ptr<Geometry> geo_) : AccelStructure(geo_) {
  root = separateGeometryBvh(geo_->fs);
}

Intersection AccelBvh::intersect(Ray ray) {
  return root->traverse(ray);
}

bool intersectBox(Vector3d m, Vector3d p, Ray ray) {
  double t_max = std::numeric_limits<double>::infinity();
  double t_min = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    double t1 = (m[i] - ray.src[i]) / ray.dir[i];
    double t2 = (p[i] - ray.src[i]) / ray.dir[i];
    double t_far = std::max(t1, t2);
    double t_near = std::min(t1, t2);
    t_max = std::min(t_max, t_far);
    t_min = std::max(t_min, t_near);

    if (t_max < t_min) {
      return false;
    }
  }
  return true;
}

Intersection AccelNode::traverse(Ray ray) {
  Intersection it;
  double min_t = std::numeric_limits<double>::infinity();
  if (children.size() == 0) { // it's a leef
    for (auto f = faces.begin(); f != faces.end(); ++f) {
      Intersection ittmp = Geometry::intersectTriangle(*f, ray);
      if (ittmp.hit && ittmp.t < min_t) {
        min_t = ittmp.t;
        it = ittmp;
      }
    }
  } else {
    for (int i = 0; i < children.size(); i++) {
      if (intersectBox(children[i]->m, children[i]->p, ray)) {
        Intersection ittmp = children[i]->traverse(ray);
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
