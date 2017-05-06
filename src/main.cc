#include <iostream>
#include <string>
#include "parser.h"
#include "geometry.h"
#include "image.h"
#include "raytrace.h"
#include "vector3d.h"
#include "exception.h"
#include "accel.h"

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cout << "usage: " << argv[0] << " [path-to-obj-file] [film-w] [film-h] [film-res]" << std::endl;
    return 1;
  }
  try {
    std::cout << "Parsing " << argv[1] << " ..." << std::endl;
    Geometry geo = ParseObj(argv[1]);
    Vector3d c = geo.center();
    geo.dump();
    std::cout << "Generating camera ..." << std::endl;
    bool st;
    Camera cam = ParseObjxCamera(argv[1], &st);
    if (!st) {
      std::cout << "[Warning] No camera definition found in " << argv[1] << std::endl;
      Vector3d up(0, 1, 0);
      cam.up(up);
      Vector3d p = c;
      double r = geo.r();
      p.x += 10.3 * r;
      cam.p(p);
      cam.film.z = 10 * r;
    }
    cam.w((c - cam.p()).normalize());
    cam.film.w = std::stod(argv[2]);
    cam.film.h = std::stod(argv[3]);
    cam.film.res = std::stod(argv[4]);
    std::cout << cam.film.z << std::endl;
    std::cout << cam.p().toString() << std::endl;
    std::cout << cam.u().toString() << std::endl;
    std::cout << cam.v().toString() << std::endl;
    std::cout << cam.w().toString() << std::endl;

    std::cout << "Preparing ..." << std::endl;
//    AccelNaive accel(std::make_shared<Geometry>(geo));
    AccelBvh accel(std::make_shared<Geometry>(geo));
//    accel.root->dump();

    std::cout << "Raytracing ..." << std::endl;
    Image<uint8_t> img = raytrace(cam, accel);

    std::cout << "Outputting ..." << std::endl;
    img.outputPpm("out.ppm");
  } catch (Exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return 0;
}
