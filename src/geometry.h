#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <vector>
#include <memory>
#include "vector3d.h"

class Ray {
 public:
  Vector3d src;
  Vector3d dir;
};

class Intersection {
 public:
  bool hit;
  Vector3d n;

  Intersection() : hit(false) {};
  Intersection(bool hit_) : hit(hit_) {};
};

class Vertex {
 public:
  std::shared_ptr<Vector3d> p;
  std::shared_ptr<Vector3d> n; // normal
  // TODO: texture
};

class Face {
 public:
  std::vector<Vertex> vs;

  int vertexCount() { return vs.size(); }
};

class Geometry {
 public:
  std::vector<Vector3d> ps; // positions of vertices
  std::vector<Vector3d> ns; // normals at vertices
  std::vector<Face> fs; // faces
  Vector3d center;

  void dump();
  Intersection intersect(Ray r);
};

class Film {
 public:
  double w, h, z;
  double res;

  Film(double w_, double h_, double z_, double res_) : w(w_), h(h_), z(z_), res(res_) {}
  Film() : w(0.0), h(0.0), z(0.0), res(0.0) {}
};

class Camera {
 private:
  Vector3d u_;
  Vector3d v_;
  Vector3d w_;

 public:
  Vector3d p; // position
  Film film;

  Camera(Vector3d u, Vector3d v, Vector3d p_, Film film_) : u_(u), v_(v), p(p_), film(film_) {}
  Camera(double w, double h, double res, Geometry geo);
  Vector3d u() { return u_; } // y axis of the camera
  Vector3d v() { return v_; } // x axis of
  Vector3d w() { return u_.cross(v_); } // z

  void u(Vector3d u) { u_ = u; }
  void v(Vector3d v) { v_ = v; }
};

#endif
