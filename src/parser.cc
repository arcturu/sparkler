#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include "parser.h"
#include "geometry.h"
#include "vector3d.h"

std::vector<std::string> split(std::string line, char delim) {
  std::stringstream sline(line);
  std::string term;
  std::vector<std::string> terms;
  while (std::getline(sline, term, delim)) {
    terms.push_back(term);
  }
  return terms;
}

Geometry ParseObj(const char *path) {
  std::ifstream file(path);
  std::string line;
  Geometry geo;
  while (std::getline(file, line)) {
    if (line.size() < 1) {
      continue;
    }
    std::stringstream sline(line);
    std::string command;
    sline >> command;
    if (command == "v") {
      double x, y, z;
      sline >> x >> y >> z;
      geo.vs.emplace_back(x, y, z);
//    } else if (command == "vt") {
    } else if (command == "vn") {
      double x, y, z;
      sline >> x >> y >> z;
      geo.vns.emplace_back(x, y, z);
    } else if (command == "f") {
      std::string term;
      std::vector<FaceComponent> f;
      while (sline >> term) {
        // each term of "f" command consists of v/vt/vn
        FaceComponent fc;
        std::vector<std::string> tokens = split(term, '/');
        if (tokens.size() != 3) {
          throw "ParseObj: Invalid face term";
        }
        if (tokens[0].size() > 0) {
          fc.v = std::stoi(tokens[0]);
        }
        if (tokens[1].size() > 0) {
          fc.vt = std::stoi(tokens[1]);
        }
        if (tokens[2].size() > 0) {
          fc.vn = std::stoi(tokens[2]);
        }
        f.push_back(fc);
      }
      geo.fs.push_back(f);
    } else if (command.size() > 0 && command[0] == '#') {
      // skip comment
    } else {
      std::cerr << "unknown command: \"" << command << "\"" << std::endl;
    }
  }
  return geo;
}
