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

typedef enum {
  Lambertian = 0
} Material;

class Intersection {
 public:
  bool hit;
  double t;
  Vector3d p;
  Vector3d n;
  Material material;

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

class Color {
 public:
  double r, g, b;

  Color(double r_, double g_, double b_) : r(r_), g(g_), b(b_) {}
  Color(const Vector3d& v) {
    r = v.x();
    g = v.y();
    b = v.z();
  }
  Color() : r(0), g(0), b(0) {}
};

class Light {
 public:
  Vector3d p;
  double luminance;
  enum Type {
    point = 0
  } type;
  Color color;

  Light(Vector3d p_, double luminance_, enum Type type_, Color color_) : p(p_), luminance(luminance_), type(type_), color(color_) {}
  Light() : p(), luminance(0), type(point) {}
};

class AccelNode {
 public:
  std::vector<std::unique_ptr<AccelNode>> children;
  std::vector<Face> faces;
  Vector3d p, m; // p: +xyz, n: -xyz boundery

  void dump();
  Intersection traverseLoop(const Ray& ray);
  Intersection traverse(const Ray& ray, double min_t);
};

class Geometry {
 public:
  std::vector<std::shared_ptr<Vector3d>> ps; // positions of vertices
  std::vector<std::shared_ptr<Vector3d>> ns; // normals at vertices
  std::vector<Face> fs; // faces
  Material material;
  std::unique_ptr<AccelNode> root;

  void dump();
  void prepare();
  double r();
  Vector3d center();
  Intersection intersectNaive(const Ray& ray) const;
  Intersection intersect(const Ray& ray) const;
  static Intersection intersectTriangle(const Face& f, const Ray& ray);
};

class Film {
 public:
  void fromFov(int pix_w, int pix_h, double size_x, double fov_deg);
  void fromRes(double w, double h, double z, double res);
  int xpixels() const { return pix_w_; }
  int ypixels() const { return pix_h_; }
  double width() const { return w_; }
  double height() const { return h_; }
  double resolution() const { return res_; }
  double distance() const { return z_; }

 private:
  int pix_w_, pix_h_;
  double w_, h_, z_;
  double res_;
  double fov_;
};

class Camera {
 private:
  Vector3d up_;
  Vector3d p_;
  Vector3d u_;
  Vector3d v_;
  Vector3d w_;
  Color bg_color_;

 public:
  Film film;

//  Camera(Vector3d u, Vector3d v, Vector3d p, Film film_) : p_(p), u_(u), v_(v), film(film_) {}
  Camera() {}
  Vector3d p() const { return p_; } // position
  Vector3d u() const { return u_; } // y axis of the camera
  Vector3d v() const { return v_; } // x axis of
  Vector3d w() const { return w_; } // z
  Color bgColor() const { return bg_color_; }

  void up(Vector3d up);
  void p(Vector3d p);
  void w(Vector3d w);
  void bgColor(Color c) { bg_color_ = c; }
};

class Scene {
 public:
  std::vector<Geometry> objects;
  std::vector<Light> lights;
  Camera camera;

  int vertexCount();
  int normalCount();
  int faceCount();
  void dump();
  Intersection intersect(const Ray& ray);
};

#include "accel.h"

#endif
