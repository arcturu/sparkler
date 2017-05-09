#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include "parser.h"
#include "geometry.h"
#include "vector3d.h"
#include "exception.h"
#include "logger.h"

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
//        for (int i = 0; i < tokens.size(); i++) {
//          std::cout << "\"" << tokens[i] << "\" ";
//        }
//        std::cout << std::endl;
        if (tokens.size() > 3 || tokens.size() == 0) {
          THROW_EXCEPTION("Invalid face term");
        }
        if (tokens[0].size() > 0) {
          v.p = std::make_shared<Vector3d>(geo.ps[std::stoi(tokens[0])-1]);
        }
        if (tokens.size() >= 2 && tokens[1].size() > 0) {
//          fc.vt = std::stoi(tokens[1]); // TODO
        }
        if (tokens.size() >= 3 && tokens[2].size() > 0) {
          v.n = std::make_shared<Vector3d>(geo.ns[std::stoi(tokens[2])-1]);
        }
        f.vs.push_back(v);
      }
      geo.fs.push_back(f);
    } else if (command == "#objx-l") { //[extension] light; x y z luminance
      Vector3d v;
      double l;
      sline >> v.x >> v.y >> v.z >> l;
      geo.ls.emplace_back(v, l);
    } else if (command.size() > 0 && command[0] == '#') {
      // skip comment
    } else {
      Logger::warn(std::string("unknown command: \"") + command + "\"");
    }
  }
  return geo;
}

Camera ParseObjxCamera(const char *path, bool *st) {
  std::ifstream file(path);
  std::string line;
  Camera cam;
  bool bcp = false, bup = false, bd = false;
  while (std::getline(file, line)) {
    if (line.size() < 1) {
      continue;
    }
    std::stringstream sline(line);
    std::string command;
    sline >> command;
    if (command == "#objx-cp") { // [extension] camera position
      Vector3d v;
      sline >> v.x >> v.y >> v.z;
      cam.p(v);
      bcp = true;
    } else if (command == "#objx-up") { // [extension] up direction
      Vector3d v;
      sline >> v.x >> v.y >> v.z;
      cam.up(v);
      bup = true;
    } else if (command == "#objx-d") { // [extension] film depth
      double d;
      sline >> d;
      cam.film.z = d;
      bd = true;
    }
  }
  *st = bcp & bup & bd;
  return cam;
}
