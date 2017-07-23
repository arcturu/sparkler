#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <vector>
#include <memory>
#include "vector3d.h"
#include "image.h"

class Ray {
 public:
  Vector3d src;
  Vector3d dir;
};

typedef enum {
  Diffuse = 0,
  Mirror = 1,
  Glass = 2,
  Glossy = 3,
  Hair = 4,
} Material;

class Intersection {
 public:
  bool hit;
  double t;
  Vector3d p;
  Vector3d n;
  Material material;
  double eta;
  Color color;

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

// Axis-Aligned Bounding Box
class Aabb {
 public:
  Vector3d p, m; // plus minus TODO: fix name

  Aabb() {}
  Aabb(Vector3d p, Vector3d m) : p(p), m(m) {}
  double volume() {
    return (p.x() - m.x()) * (p.y() - m.y()) * (p.z() - m.z());
  }
  void merge(const Aabb& other);
  void dump() {
    printf("[%f %f %f] [%f %f %f]\n", m.x(), m.y(), m.z(), p.x(), p.y(), p.z());
  }
};

class Object {
 public:
  Material material;
  double eta;
  Color color;

  virtual Intersection intersect(const Ray& ray) = 0;
  virtual Aabb getAabb() = 0;
  virtual bool belongsTo(const Aabb& box) = 0;
};

/*
class Triangle : public Object {
  std::vector<Vector3d> vs; // vertices
  std::vector<Vector3d> ns; // normals
}
*/

class Sphere : public Object {
  Vector3d c;
  double r;
 public:
  Sphere(Vector3d c, double r, Material m, double e, Color col) : c(c), r(r) {
    material = m;
    eta = e;
    color = col;
  }
  Intersection intersect(const Ray& ray);
  virtual Aabb getAabb();
  virtual bool belongsTo(const Aabb& box);
};

class Cylinder : public Object {
 private:
  Vector3d src;
  Vector3d dir;
  double len;
  double r;
 public:
  Cylinder(Vector3d s, Vector3d d, double l, double r, Material m, double e, Color c) : src(s), dir(d), len(l), r(r) {
    material = m;
    eta = e;
    color = c;
  }
  Intersection intersect(const Ray& ray);
  virtual Aabb getAabb();
  virtual bool belongsTo(const Aabb& box);
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

class BvhNode {
 public:
  std::vector<std::unique_ptr<BvhNode>> children;
  std::vector<std::unique_ptr<Object>> objects;
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
  double eta; // refractive index
  std::unique_ptr<AccelNode> root;
  std::unique_ptr<BvhNode> obj_root;

  std::vector<std::unique_ptr<Object>> objects;
  double curve_thickness;
  double curve_transparency;

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
  Image<double> bg_image_;

 public:
  Film film;

//  Camera(Vector3d u, Vector3d v, Vector3d p, Film film_) : p_(p), u_(u), v_(v), film(film_) {}
  Camera() {}
  Vector3d p() const { return p_; } // position
  Vector3d u() const { return u_; } // y axis of the camera
  Vector3d v() const { return v_; } // x axis of
  Vector3d w() const { return w_; } // z
  Color bgColor() const { return bg_color_; }
  const Image<double>& bgImage() const { return bg_image_; }

  void up(Vector3d up);
  void p(Vector3d p);
  void w(Vector3d w);
  void bgColor(Color c) { bg_color_ = c; }
  void bgImage(const Image<double>& img) { bg_image_ = img; }
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
