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
  double t;
  Vector3d p;
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

  int vertexCount() const { return vs.size(); }
};

class Light {
 public:
  Vector3d p;
  double luminance;

  Light(Vector3d p_, double luminance_) : p(p_), luminance(luminance_) {}
  Light() : p(), luminance(0) {}
};

class Geometry {
 public:
  std::vector<Vector3d> ps; // positions of vertices
  std::vector<Vector3d> ns; // normals at vertices
  std::vector<Face> fs; // faces
  std::vector<Light> ls; // lights

  void dump();
  double r();
  Vector3d center();
  static Intersection intersectTriangle(const Face& f, const Ray& ray);
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
  Vector3d up_;
  Vector3d p_;
  Vector3d u_;
  Vector3d v_;
  Vector3d w_;

 public:
  Film film;

//  Camera(Vector3d u, Vector3d v, Vector3d p, Film film_) : p_(p), u_(u), v_(v), film(film_) {}
  Camera() {}
  void dump();
  Vector3d p() { return p_; } // position
  Vector3d u() { return u_; } // y axis of the camera
  Vector3d v() { return v_; } // x axis of
  Vector3d w() { return w_; } // z

  void up(Vector3d up);
  void p(Vector3d p);
  void w(Vector3d w);
};

#endif
