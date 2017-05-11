#include <cmath>
#include <memory>
#include <immintrin.h>
#include "accel.h"
#include "constant.h"
#include "exception.h"
#include "geometry.h"

#include <iostream>

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
  std::unique_ptr<AccelNode> n(new AccelNode);
  const int SEARCH_DIV_RES = 10;
  const int MIN_OBJS = 2;
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

Intersection AccelBvh::intersect(const Ray& ray) {
  return root->traverseLoop(ray);
/*
  const double min_t = std::numeric_limits<double>::infinity();
  return root->traverse(ray, min_t);
*/
}

void vec_add(const size_t n, double *z, const double *x, const double *y){
    static const size_t double_size = 4;
    const size_t end = n / double_size;

    __m256d *vz = (__m256d *)z;
    __m256d *vx = (__m256d *)x;
    __m256d *vy = (__m256d *)y;

    for(size_t i=0; i<end; ++i)
        vz[i] = _mm256_add_pd(vx[i], vy[i]);
}

bool intersectBox(const Vector3d& m, const Vector3d& p, const Ray& ray, double *t_hit = nullptr) {
  double t_max = std::numeric_limits<double>::infinity();
  double t_min = -std::numeric_limits<double>::infinity();
  double t_far[4], t_near[4];

  *t_hit = t_max;

  __m256d *vt_far = (__m256d *)t_far;
  __m256d *vt_near = (__m256d *)t_near;
  __m256d *vm = (__m256d *)m.p;
  __m256d *vp = (__m256d *)p.p;
  __m256d *vrs = (__m256d *)ray.src.p;
  __m256d *vrd = (__m256d *)ray.dir.p;
  __m256d vt1 = _mm256_div_pd(_mm256_sub_pd(*vm, *vrs), *vrd);
  __m256d vt2 = _mm256_div_pd(_mm256_sub_pd(*vp, *vrs), *vrd);
  *vt_far = _mm256_max_pd(vt1, vt2);
  *vt_near = _mm256_min_pd(vt1, vt2);


//  std::cout << m.toString() << p.toString() << ray.src.toString() << ray.dir.toString() << std::endl;
//  for (int i = 0; i < 4; i++) {
//    std::cout << t1[i] << " ";
//  }
//  std::cout << std::endl;
//  for (int i = 0; i < 4; i++) {
//    std::cout << t2[i] << " ";
//  }
//  std::cout << std::endl;

  for (int i = 0; i < 3; i++) {
//    t1 = (m.p[i] - ray.src.p[i]) / ray.dir.p[i];
//    t2 = (p.p[i] - ray.src.p[i]) / ray.dir.p[i];
//    t_far = std::max(t1[i], t2[i]);
//    t_near = std::min(t1[i], t2[i]);
    t_max = std::min(t_max, t_far[i]);
    t_min = std::max(t_min, t_near[i]);

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
  for (int i = 0; i < children.size(); i++) {
    if (intersectBox(children[i]->m, children[i]->p, ray, &t)) {
      ns.push(children[i].get());
    }
  }
  while (!ns.empty()) {
    // TODO: improve search order
    AccelNode *n = ns.top(); ns.pop();
    if (n->children.size() == 0) { // n is a leef node
      for (int i = 0; i < n->faces.size(); i++) {
        Intersection tmp_it = Geometry::intersectTriangle(n->faces[i], ray);
        if (tmp_it.hit && tmp_it.t < min_t) {
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
