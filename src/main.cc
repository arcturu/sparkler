#include <iostream>
#include <string>
#include "parser.h"
#include "geometry.h"
#include "image.h"
#include "raytrace.h"
#include "vector3d.h"
#include "exception.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " [path-to-obj-file]" << std::endl;
  }
  try {
    std::cout << "Parsing " << argv[1] << " ..." <<std::endl;
    Geometry geo = ParseObj(argv[1]);
    geo.dump();
    std::cout << "Generating camera ..." << std::endl;
    Camera camera(3, 3, 0.01, geo);
    std::cout << camera.film.z << std::endl;
    std::cout << camera.p.toString() << std::endl;
    std::cout << camera.u().toString() << std::endl;
    std::cout << camera.v().toString() << std::endl;
    std::cout << camera.w().toString() << std::endl;

    std::cout << "Raytracing ..." << std::endl;
    Image<uint8_t> img = raytrace(camera, geo);
    std::cout << "Outputting ..." << std::endl;
    img.outputPpm("out.ppm");
  } catch (Exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return 0;
}
