#include <iostream>
#include "parser.h"
#include <string>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " [path-to-obj-file]" << std::endl;
  }
  Geometry geo = ParseObj(argv[1]);
  geo.dump();
  return 0;
}
