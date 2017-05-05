#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include "parser.h"
#include "geometry.h"
#include "vector3d.h"
#include "exception.h"

std::vector<std::string> split(std::string line, char delim) {
  std::vector<std::string> terms;

  int from = 0;
  for (int i = 0; i < line.size(); ++i) {
    if (line[i] == delim) {
      terms.push_back(line.substr(from, i - from));
      from = i + 1;
    }
  }
  terms.push_back(line.substr(from)); // to end

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
      Vector3d v(x, y, z);
      geo.ps.push_back(v);
      geo.center = geo.center + v;
//    } else if (command == "vt") {
    } else if (command == "vn") {
      double x, y, z;
      sline >> x >> y >> z;
      Vector3d n(x, y, z);
      geo.ns.push_back(n.normalize());
    } else if (command == "f") {
      std::string term;
      Face f;
      while (sline >> term) {
        // each term of "f" command consists of v/vt/vn
        Vertex v;
        std::vector<std::string> tokens = split(term, '/');
        if (tokens.size() != 3) {
          std::cout << std::endl;
          THROW_EXCEPTION("ParseObj: Invalid face term");
        }
        if (tokens[0].size() > 0) {
          v.p = std::make_shared<Vector3d>(geo.ps[std::stoi(tokens[0])-1]);
        }
        if (tokens[1].size() > 0) {
//          fc.vt = std::stoi(tokens[1]); // TODO
        }
        if (tokens[2].size() > 0) {
          v.n = std::make_shared<Vector3d>(geo.ns[std::stoi(tokens[2])-1]);
        }
        f.vs.push_back(v);
      }
      geo.fs.push_back(f);
    } else if (command.size() > 0 && command[0] == '#') {
      // skip comment
    } else {
      std::cerr << "unknown command: \"" << command << "\"" << std::endl;
    }
  }
  geo.center = geo.center / geo.ps.size();
  return geo;
}
