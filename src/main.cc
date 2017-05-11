#include <iostream>
#include <string>
#include <chrono>
#include "parser.h"
#include "geometry.h"
#include "image.h"
#include "raytrace.h"
#include "vector3d.h"
#include "exception.h"
#include "accel.h"
#include "logger.h"

#define MS(x) std::chrono::duration_cast<std::chrono::milliseconds>(x).count()

typedef std::chrono::high_resolution_clock Clock;

int main(int argc, char **argv) {
  if (argc != 5) {
    Logger::info(std::string("usage: ") + argv[0] + " [path-to-obj-file] [film-w] [film-h] [film-res]");
    return 1;
  }
  try {
    Logger::info(std::string("Parsing ") + argv[1]);
    Geometry geo = ParseObj(argv[1]);
    Vector3d c = geo.center();
    geo.dump();
    if (geo.ls.size() == 0) {
      Logger::warn("No light definition found");
    }
    bool st;
    Camera cam = ParseObjxCamera(argv[1], &st);
    if (!st) {
      Logger::warn("No camera definition found");
      Vector3d up(0, 1, 0);
      cam.up(up);
      Vector3d p = c;
      double r = geo.r();
      p.x(p.x() + 10.3 * r);
      cam.p(p);
      cam.film.z = 10 * r;
    }
    cam.w((c - cam.p()).normalize());
    cam.film.w = std::stod(argv[2]);
    cam.film.h = std::stod(argv[3]);
    cam.film.res = std::stod(argv[4]);
    cam.dump();
//    std::cout << cam.film.z << std::endl;
//    std::cout << cam.p().toString() << std::endl;
//    std::cout << cam.u().toString() << std::endl;
//    std::cout << cam.v().toString() << std::endl;
//    std::cout << cam.w().toString() << std::endl;

    Logger::info("Preparing accel structure");
//    AccelNaive accel(std::make_shared<Geometry>(geo));
    auto t1 = Clock::now();
    AccelBvh accel(std::make_shared<Geometry>(geo));
    auto t2 = Clock::now();
    Logger::info(std::string("Preparing accel structure finished in ") + std::to_string(MS(t2 - t1)) + " ms");
//    accel.root->dump();

    Logger::info("Raytracing");
    t1 = Clock::now();
    Image<uint8_t> img = raytrace(cam, geo, accel);
    t2 = Clock::now();
    int time = MS(t2 - t1);
    Logger::info(std::string("Raytracing finished in ") + std::to_string(time) + " ms (" + std::to_string((double)img.width() * img.height() / time * 1000 / 1000 / 1000) + " Mrps)");

    Logger::info("Outputting");
    img.outputPpm("out.ppm");
  } catch (Exception& e) {
    Logger::error(e.what());
  }
  return 0;
}
