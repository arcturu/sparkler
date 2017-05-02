#include <iostream>
#include "geometry.h"

void Geometry::dump() {
  // dump statistics
  std::cout << "#vertices: " << this->vs.size() << std::endl;
  std::cout << "#normals: " << this->vns.size() << std::endl;
  std::cout << "#faces: " << this->fs.size() << std::endl;
}
