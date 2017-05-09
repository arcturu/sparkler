#include <iostream>
#include <cmath>
#include <limits>
#include <memory>
#include "geometry.h"
#include "exception.h"
#include "constant.h"

void Geometry::dump() {
  // dump statistics
  std::cout << "#vertices: " << ps.size() << std::endl;
  std::cout << " #normals: " << ns.size() << std::endl;
  std::cout << "   #faces: " << fs.size() << std::endl;
  std::cout << "   center: " << center().toString() << std::endl;
  std::cout << "        r: " << r() << std::endl;
  for (auto l = ls.begin(); l != ls.end(); ++l) {
    std::cout << "    light: " << "at " << l->p.toString() << " luminance = " << l->luminance << std::endl;
  }
}

void Camera::dump() {
  std::cout << "   camera: p = " << p().toString() << std::endl;
  std::cout << "   camera: u = " << u().toString() << std::endl;
  std::cout << "   camera: v = " << v().toString() << std::endl;
  std::cout << "   camera: w = " << w().toString() << std::endl;
  std::cout << "   camera: d = " << film.z << std::endl;
}

double Geometry::r() {
  double r = 0;
  Vector3d c = center();
  for (auto v = ps.begin(); v !=  ps.end(); ++v) {
    double dist = (*v - c).length();
    if (dist > r) {
      r = dist;
    }
  }
  return r;
}

Vector3d Geometry::center() {
  Vector3d c;
  for (auto v = ps.begin(); v != ps.end(); ++v) {
    c = c + *v;
  }
  c = c / ps.size();
  return c;
}

Intersection Geometry::intersectTriangle(const Face& f, const Ray& ray) {
  Intersection it;
  if (f.vertexCount() != 3) {
    THROW_EXCEPTION("Not implemented yet");
  }
  // ray-plane intersection (f->vs is assumed to be arranged in counter clock-wise)
  Vector3d plane_n = (*f.vs[1].p - *f.vs[0].p).cross(*f.vs[2].p - *f.vs[0].p);
  double plane_d = -plane_n.dot(*f.vs[0].p);
  double vn = ray.dir.dot(plane_n);
  if (std::abs(vn) < Constant::EPS) {
    // the ray is parallel to the plane
    return it;
  }
  double t = -(ray.src.dot(plane_n) + plane_d) / vn;
  Vector3d q = ray.src + t * ray.dir;

  // check if the intersection point is inside triangle
  double va = (q - ray.src).dot((*f.vs[2].p - ray.src).cross(*f.vs[1].p - ray.src));
  double vb = (q - ray.src).dot((*f.vs[0].p - ray.src).cross(*f.vs[2].p - ray.src));
  double vc = (q - ray.src).dot((*f.vs[1].p - ray.src).cross(*f.vs[0].p - ray.src));

  if ((va >= 0 && vb >= 0 && vc >= 0) || (va <= 0 && vb <= 0 && vc <= 0)) {
    double vsum = va + vb + vc;
    it.hit = true;
    it.t = t;
    it.p = q;
    if (f.vs[0].n && f.vs[1].n && f.vs[2].n) {
      it.n = (va / vsum * *f.vs[0].n + vb / vsum * *f.vs[1].n + vc / vsum * *f.vs[2].n).normalize();
    } else {
      it.n = -plane_n.normalize();
    }
  }
  return it;
}

void Camera::up(Vector3d up) {
  up_ = up;
  v_ = w_.cross(up_);
  u_ = v_.cross(w_);
}

void Camera::p(Vector3d p) {
  p_ = p;
}

void Camera::w(Vector3d w) {
  w_ = w;
  v_ = w_.cross(up_);
  u_ = v_.cross(w_);
}
