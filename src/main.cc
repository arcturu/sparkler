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
  if (argc != 2) {
    Logger::info(std::string("usage: ") + argv[0] + " [ path-to-obj-file ]");
    return 1;
  }
  try {
    Logger::info(std::string("Parsing ") + argv[1]);
    Scene scene = ParseScene(argv[1]);
    scene.dump();

    Logger::info("Raytrace");
    auto t1 = Clock::now();
    Image<uint8_t> img = raytrace(scene);
    auto t2 = Clock::now();
    int time = MS(t2 - t1);
    Logger::info(std::string("Raytrace finished in ") + std::to_string(time) + " ms (" + std::to_string((double)img.width() * img.height() / time * 1000 / 1000 / 1000) + " Mrps)");

    Logger::info("Output");
    img.outputPpm("out.ppm");
  } catch (Exception& e) {
    Logger::error(e.what());
  }
  return 0;
}
