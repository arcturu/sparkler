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
#include "stat.h"
#include "global.h"

#define MS(x) std::chrono::duration_cast<std::chrono::milliseconds>(x).count()

typedef std::chrono::high_resolution_clock Clock;

void initStat() {
  stat_num_intersectTriangle = 0;
  stat_num_intersectBox = 0;
  stat_num_traverse = 0;
  stat_num_accel_node = 0;
  stat_num_accel_leaf = 0;
  stat_num_ray = 0;
}

void dumpStat() {
  std::cout << "stat #it-tri   : " << stat_num_intersectTriangle << std::endl;
  std::cout << "stat #it-box   : " << stat_num_intersectBox << std::endl;
  std::cout << "stat #traverse : " << stat_num_traverse << std::endl;
  std::cout << "stat #anode    : " << stat_num_accel_node << std::endl;
  std::cout << "stat #aleaf    : " << stat_num_accel_leaf << std::endl;
  std::cout << "stat #ray      : " << stat_num_ray << std::endl;
  std::cout << std::endl;
  std::cout << "G_MIN_OBJS       : " << G_MIN_OBJS << std::endl;
  std::cout << "G_SEARCH_DIV_RES : " << G_SEARCH_DIV_RES << std::endl;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    Logger::info(std::string("usage: ") + argv[0] + " [ path-to-scene-file ] [ obj-dir ]");
    return 1;
  }
  try {
    initStat();
    Logger::info(std::string("Parsing ") + argv[1]);
    auto t1 = Clock::now();
    Scene scene = ParseScene(argv[1], argv[2]);
    auto t2 = Clock::now();
    Logger::info(std::string("Parse & prepare finished in ") + std::to_string(MS(t2 - t1)) + " ms");
    scene.dump();

    Logger::info("Raytrace");
    t1 = Clock::now();
    Image<uint8_t> img = raytrace(scene);
    t2 = Clock::now();
    int time = MS(t2 - t1);
    Logger::info(std::string("Raytrace finished in ") + std::to_string(time) + " ms (" + std::to_string((double)img.width() * img.height() / time * 1000 / 1000 / 1000) + " Mrps)");

    Logger::info("Output");
    img.outputPpm("out.ppm");

    dumpStat();
  } catch (Exception& e) {
    Logger::error(e.what());
  }
  return 0;
}
