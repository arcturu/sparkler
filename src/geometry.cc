#include <iostream>
#include <cmath>
#include <limits>
#include "geometry.h"
#include "exception.h"
#include "constant.h"

void Geometry::dump() {
  // dump statistics
  std::cout << "#vertices: " << ps.size() << std::endl;
  std::cout << "#normals: " << ns.size() << std::endl;
  std::cout << "#faces: " << fs.size() << std::endl;
}

Intersection Geometry::intersect(Ray ray) {
  Intersection it;
  double min_t = std::numeric_limits<double>::infinity();

  for (auto f = fs.begin(); f != fs.end(); ++f) {
    if (f->vertexCount() != 3) {
      THROW_EXCEPTION("Not implemented yet");
    }
    // ray-plane intersection (f->vs is assumed to be arranged in counter clock-wise)
    Vector3d plane_n = (*f->vs[1].p - *f->vs[0].p).cross(*f->vs[2].p - *f->vs[0].p);
    double plane_d = -plane_n.dot(*f->vs[0].p);
    double vn = ray.dir.dot(plane_n);
    if (std::abs(vn) < Constant::EPS) {
      // the ray is parallel to the plane
      continue;
    }
    double t = -(ray.src.dot(plane_n) + plane_d) / vn;
    Vector3d q = ray.src + t * ray.dir;
//    std::cout << q.toString() << std::endl;

    // check if the intersection point is inside triangle
    double va = (q - ray.src).dot((*f->vs[2].p - ray.src).cross(*f->vs[1].p - ray.src));
    double vb = (q - ray.src).dot((*f->vs[0].p - ray.src).cross(*f->vs[2].p - ray.src));
    double vc = (q - ray.src).dot((*f->vs[1].p - ray.src).cross(*f->vs[0].p - ray.src));

//    std::cout << va << " " << vb << " " << vc << std::endl;
    if ((va >= 0 && vb >= 0 && vc >= 0) || (va <= 0 && vb <= 0 && vc <= 0)) {
      if (t >= 0 && t < min_t) {
        min_t = t;
        double vsum = va + vb + vc;
        it.hit = true;
        it.p = q;
        if (f->vs[0].n && f->vs[1].n && f->vs[2].n) {
          it.n = (va / vsum * *f->vs[0].n + vb / vsum * *f->vs[1].n + vc / vsum * *f->vs[2].n).normalize();
        } else {
          it.n.x = 1.0; it.n.y = 0; it.n.z = 0;
        }
      }
    }
  }

  return it;
}

// calculate feasible camera position from the geometry
Camera::Camera(double w, double h, double res, Geometry geo) {
  double r = 0;
  for (auto v = geo.ps.begin(); v !=  geo.ps.end(); ++v) {
    double dist = (*v - geo.center).length();
    if (dist > r) {
      r = dist;
    }
  }
  p = geo.center;
  p.z += r * 10;
  film.w = w; film.h = h; film.z = 10.0; film.res = res;
  w_ = (geo.center - p).normalize();
  Vector3d up(0, 1, 0);
  v_ = w_.cross(up);
  u_ = v_.cross(w_);
}
