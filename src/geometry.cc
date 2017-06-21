#include <iostream>
#include <cmath>
#include <limits>
#include <memory>
#include "geometry.h"
#include "exception.h"
#include "constant.h"
#include "accel.h"
#include "stat.h"

void Scene::dump() {
  // dump statistics
  std::cout << "#vertices: " << vertexCount() << std::endl;
  std::cout << " #normals: " << normalCount() << std::endl;
  std::cout << "   #faces: " << faceCount() << std::endl;
//  std::cout << "   center: " << center().toString() << std::endl;
//  std::cout << "        r: " << r() << std::endl;
  for (const auto& l : lights) {
    std::cout << "    light: " << "at " << l.p.toString() << " luminance = " << l.luminance << " (r, g, b) = (" << l.color.r << ", " << l.color.g << ", " << l.color.b << ")" << std::endl;
  }
  std::cout << "   camera: p = " << camera.p().toString() << std::endl;
  std::cout << "   camera: u = " << camera.u().toString() << std::endl;
  std::cout << "   camera: v = " << camera.v().toString() << std::endl;
  std::cout << "   camera: w = " << camera.w().toString() << std::endl;
  std::cout << "   camera: d = " << camera.film.distance() << std::endl;
}

Intersection Scene::intersect(const Ray& ray) {
  Intersection tmp_it;
  Intersection min_it;
  min_it.t = std::numeric_limits<double>::infinity();
  for (const auto& object : objects) {
    tmp_it = object.intersect(ray);
    if (tmp_it.hit && tmp_it.t >= 0 && min_it.t > tmp_it.t) {
      min_it = tmp_it;
    }
  }
  return min_it;
}

int Scene::vertexCount() {
  int count = 0;
  for (const auto& obj : objects) {
    count += obj.ps.size();
  }
  return count;
}

int Scene::normalCount() {
  int count = 0;
  for (const auto& obj : objects) {
    count += obj.ns.size();
  }
  return count;
}

int Scene::faceCount() {
  int count = 0;
  for (const auto& obj : objects) {
    count += obj.fs.size();
  }
  return count;
}

void Geometry::dump() {
  std::cout << "#vertices: " << ps.size() << std::endl;
  std::cout << " #normals: " << ns.size() << std::endl;
  std::cout << "   #faces: " << fs.size() << std::endl;
  std::cout << "   center: " << center().toString() << std::endl;
  std::cout << "        r: " << r() << std::endl;
}

double Geometry::r() {
  double r = 0;
  Vector3d c = center();
  for (const auto& p : ps) {
    double dist = (*p - c).length();
    if (dist > r) {
      r = dist;
    }
  }
  return r;
}

Vector3d Geometry::center() {
  Vector3d c;
  for (const auto& p : ps) {
    c = c + *p;
  }
  c = c / ps.size();
  return c;
}

Intersection Geometry::intersectTriangle(const Face& f, const Ray& ray) {
  stat_num_intersectTriangle++;
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
      it.n = plane_n.normalize();
    }
  }
  return it;
}

Intersection Geometry::intersectNaive(const Ray& ray) const {
  Intersection it;
  double min_t = std::numeric_limits<double>::infinity();

  for (const auto& f : fs) {
    Intersection ittmp = Geometry::intersectTriangle(f, ray);
    if (ittmp.hit && ittmp.t >= 0 && ittmp.t < min_t) {
      min_t = ittmp.t;
      it = ittmp;
    }
  }

  return it;
}


Intersection Geometry::intersect(const Ray& ray) const {
  Intersection it = root->traverseLoop(ray);
  if (it.hit) {
    it.material = material;
    it.eta = eta;
  }
  return it;
}

void Geometry::prepare() {
  root = separateGeometryBvh(fs);
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

void Film::fromFov(int pix_w, int pix_h, double size_x, double fov_deg) {
  pix_w_ = pix_w;
  pix_h_ = pix_h;
  fov_ = fov_deg / 360 * 2 * M_PI;
  w_ = size_x;

  z_ = size_x / 2 / tan(fov_ / 2);
  res_ = size_x / pix_w;
  h_ = res_ * pix_h;
}

void Film::fromRes(double w, double h, double z, double res) {
  w_ = w;
  h_ = h;
  z_ = z;
  res_ = res;

  pix_w_ = w_ / res_;
  pix_h_ = h_ / res_;
  fov_ = 2 * atan(w_ / 2 / z_);
}
