#include <iostream>
#include <cmath>
#include <limits>
#include <memory>
#include "geometry.h"
#include "exception.h"
#include "constant.h"
#include "accel.h"
#include "stat.h"
#include "logger.h"

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
      if (it.n.x() != it.n.x()) {
        it.n = plane_n.normalize();
//        printf("hoe\n");
//        printf("%f %f %f\n", va, vb, vc);
//        Logger::info(it.n.toString() + f.vs[0].n->toString() + f.vs[1].n->toString() + f.vs[2].n->toString());
      }
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

//  Intersection it2 = obj_root->traverseLoop(ray);
//  if (it2.hit && (!it.hit || it2.t < it.t)) {
//    it = it2;
//  }
  for (auto& obj : objects) {
    Intersection it2 = obj->intersect(ray);
    if (it2.hit && (!it.hit || it2.t < it.t)) {
      it = it2;
    }
  }
  return it;
}

void Geometry::prepare() {
  root = separateGeometryBvh(fs);
//  obj_root = constructBvh(std::move(objects));
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

Intersection Sphere::intersect(const Ray& ray) {
  Vector3d v = ray.src - c;
  double A = ray.dir.dot(ray.dir);
  double B = 2 * ray.dir.dot(v);
  double C = v.dot(v) - r * r;

  Intersection it;
  double det = B * B - 4 * A * C;
  if (det < 0 || A == 0) { // A == 0 => B == 0
    return it;
  }
  int num_candidates = 2;
  double t[] = {(-B + sqrt(det)) / (2 * A), (-B - sqrt(det)) / (2 * A)};
  it.t = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_candidates; i++) {
    if (t[i] >= 0 && t[i] < it.t) {
      it.hit = true;
      it.t = t[i];
      it.p = ray.src + t[i] * ray.dir;
      it.n = (it.p - c).normalize();
      it.material = material;
      it.eta = eta;
    }
  }
  return it;
}

Aabb Sphere::getAabb() {
  Vector3d rvec(r, r, r);
  return Aabb(c + rvec, c - rvec);
}

// sphere-aabb intersection
// https://studiofreya.com/3d-math-and-physics/sphere-vs-aabb-collision-detection-test/
double distPointAabb1d(double p, double bmin, double bmax) {
  if (p < bmin) {
    return pow(bmin - p, 2);
  }
  if (p > bmax) {
    return pow(p - bmax, 2);
  }
  return 0;
}
double distPointAabb(const Vector3d& p, const Aabb& box) {
  double sq = 0;
  sq += distPointAabb1d(p.x(), box.m.x(), box.p.x());
  sq += distPointAabb1d(p.y(), box.m.y(), box.p.y());
  sq += distPointAabb1d(p.z(), box.m.z(), box.p.z());
  return sq;
}
bool Sphere::belongsTo(const Aabb& box) {
  if (distPointAabb(c, box) <= r * r) {
    return true;
  } else {
    return false;
  }
}

// http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf
Intersection Cylinder::intersect(const Ray& ray) {
  Vector3d v = ray.dir - ray.dir.dot(dir) * dir;
  Vector3d w = ray.src - src - (ray.src - src).dot(dir) * dir;
  double A = v.dot(v);
  double B = 2 * v.dot(w);
  double C = w.dot(w) - r * r;

  Intersection it;
  double det = B * B - 4 * A * C;
  if (det < 0 || A == 0) { // A == 0 => B == 0
    return it;
  }
  int num_candidates = 2;
  double t[] = {(-B + sqrt(det)) / (2 * A), (-B - sqrt(det)) / (2 * A)};
  it.t = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_candidates; i++) {
    if (dir.dot(ray.src + t[i] * ray.dir - src) > 0 &&
        dir.dot(ray.src + t[i] * ray.dir - (src + len * dir)) < 0 &&
        t[i] >= 0 && t[i] < it.t) {
      it.hit = true;
      it.t = t[i];
      it.p = ray.src + t[i] * ray.dir;
      it.n = (it.p - (src + dir.dot(ray.src + t[i] * ray.dir - src) * dir)).normalize();
      it.material = material;
      it.eta = eta;
    }
  }
  return it;
}

Aabb Cylinder::getAabb() {
  // using circumscribed sphere
  // TODO more precisely
  Vector3d c = src + (len / 2) * dir.normalize();
  Vector3d n = dir.cross(dir.normalize() + Vector3d(0.1, 0, 0)).normalize();
  double d = (c - (src + r * n)).length();
  Vector3d dvec(d, d, d);
  return Aabb(c + dvec, c - dvec);
}

bool Cylinder::belongsTo(const Aabb& box) {
  // using circumscribed sphere
  // TODO more precisely
  Vector3d c = src + (len / 2) * dir.normalize();
  Vector3d n = dir.cross(dir.normalize() + Vector3d(0.1, 0, 0)).normalize();
  double d = (c - (src + r * n)).length();
  if (distPointAabb(c, box) <= d * d) {
    return true;
  } else {
    return false;
  }
}

void Aabb::merge(const Aabb& other) {
  if (p.x() < other.p.x()) { p.x(other.p.x()); }
  if (p.y() < other.p.y()) { p.y(other.p.y()); }
  if (p.z() < other.p.z()) { p.z(other.p.z()); }
  if (m.x() > other.m.x()) { m.x(other.m.x()); }
  if (m.y() > other.m.y()) { m.y(other.m.y()); }
  if (m.z() > other.m.z()) { m.z(other.m.z()); }
}
