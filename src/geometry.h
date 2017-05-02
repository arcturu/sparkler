#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <vector>
#include "vector3d.h"

class FaceComponent {
 public:
  int v; // index of vector
  int vt; // index of vector texture
  int vn; // index of vector normal
};

class Geometry {
 public:
  std::vector<Vector3d> vs; // vertices
  std::vector<Vector3d> vns; // normal @ vertices
  std::vector<std::vector<FaceComponent>> fs; // faces

  void dump();
};

#endif
