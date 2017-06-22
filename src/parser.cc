#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include "parser.h"
#include "geometry.h"
#include "accel.h"
#include "vector3d.h"
#include "exception.h"
#include "logger.h"
#include "global.h"
#include "../lib/json11/json11.hpp"

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

Geometry ParseObj(const std::string path) {
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
      geo.ps.push_back(std::make_shared<Vector3d>(x, y, z));
//    } else if (command == "vt") {
    } else if (command == "vn") {
      double x, y, z;
      sline >> x >> y >> z;
      Vector3d n(x, y, z);
      geo.ns.push_back(std::make_shared<Vector3d>(n.normalize()));
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
          v.p = geo.ps[std::stoi(tokens[0])-1];
        }
        if (tokens.size() >= 2 && tokens[1].size() > 0) {
//          fc.vt = std::stoi(tokens[1]); // TODO
        }
        if (tokens.size() >= 3 && tokens[2].size() > 0) {
          v.n = geo.ns[std::stoi(tokens[2])-1];
        }
        f.vs.push_back(v);
      }
      geo.fs.push_back(f);
    } else if (command.size() > 0 && command[0] == '#') {
      // skip comment
    } else {
//      Logger::warn(std::string("unknown command: \"") + command + "\"");
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
      double x, y, z;
      sline >> x >> y >> z;
      Vector3d v(x, y, z);
      cam.p(v);
      bcp = true;
    } else if (command == "#objx-up") { // [extension] up direction
      double x, y, z;
      sline >> x >> y >> z;
      Vector3d v(x, y, z);
      cam.up(v);
      bup = true;
//    } else if (command == "#objx-d") { // [extension] film depth
//      double d;
//      sline >> d;
//      cam.film.z = d;
//      bd = true;
    }
  }
  *st = bcp & bup & bd;
  return cam;
}

Vector3d to_v3d(const json11::Json& json) {
  Vector3d v;
  for (int i = 0; i < 3; i++) {
    v[i] = json[i].number_value();
  }
  return v;
}

template <typename T>
Vector3d to_v3d(const T v) {
  Vector3d v3;
  for (int i = 0; i < 3; i++) {
    v3[i] = v[i];
  }
  return v3;
}

Scene ParseScene(const char *path, const char *obj_dir) {
  Scene scene;
  std::ifstream ifs(path);
  std::istreambuf_iterator<char> it(ifs);
  std::istreambuf_iterator<char> last;
  std::string json_str(it, last);

  std::string err;
  json11::Json json = json11::Json::parse(json_str, err);
  if (err != "") {
    THROW_EXCEPTION(err);
  }

  // set global variables
  G_SEARCH_DIV_RES = json["bvh-resolution"].int_value();
  G_MIN_OBJS = json["bvh-min-objs"].int_value();

//  if (json["camera"]) {
    json11::Json jcam = json["camera"];
    if (jcam["size"].is_null())     { THROW_EXCEPTION("camera.size is not defined"); }
    if (jcam["pixels"].is_null())   { THROW_EXCEPTION("camera.pixels is not defined"); }
    if (jcam["fov"].is_null())      { THROW_EXCEPTION("camera.fov is not defined"); }
    if (jcam["position"].is_null()) { THROW_EXCEPTION("camera.position is not defined"); }
    if (jcam["look-at"].is_null())  { THROW_EXCEPTION("camera.look-at is not defined"); }
    if (jcam["up"].is_null())       { THROW_EXCEPTION("camera.up is not defined"); }
    if (jcam["bg-color"].is_null()) { THROW_EXCEPTION("camera.color is not defined"); }
    scene.camera.up(to_v3d(jcam["up"]));
    scene.camera.p(to_v3d(jcam["position"]));
    scene.camera.w((to_v3d(jcam["look-at"]) - scene.camera.p()).normalize());
    scene.camera.bgColor(Color(to_v3d(jcam["bg-color"])));
    scene.camera.bgImage(ParsePfm(jcam["bg-image"].string_value()));
    scene.camera.film.fromFov(jcam["pixels"][0].int_value(),
                              jcam["pixels"][1].int_value(),
                              jcam["size"].number_value(),
                              jcam["fov"].number_value());
//  }
  for (const auto& jl : json["light"].array_items()) {
    Color color(to_v3d(jl["color"]));
    scene.lights.emplace_back(to_v3d(jl["position"]),
                              jl["energy"].number_value(),
                              static_cast<Light::Type>(jl["type"].int_value()),
                              color);
  }
  for (const auto& jobj : json["geometry"].array_items()) {
    Logger::info(std::string("loading ") + jobj["file"].string_value());
    Geometry geo = ParseObj(std::string(obj_dir) + "/" + jobj["file"].string_value());
    if (!jobj["transform"].is_null()) {
      double M[4][4];
      for (int i = 0; i < 4; i++ ){
        for (int j = 0; j < 4; j++) {
          M[i][j] = jobj["transform"][i][j].number_value();
        }
      }
      for (const auto& p : geo.ps) {
        double v[4];
        for (int i = 0; i < 3; i++) {
          v[i] = (*p)[i];
          (*p)[i] = 0;
        }
        v[3] = 1;
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 4; j++) {
            (*p)[i] += M[i][j] * v[j];
          }
        }
      }
    }
    geo.dump();
    geo.prepare(); // construct accel structure
    geo.material = static_cast<Material>(jobj["material"].int_value());
    if (!jobj["eta"].is_null()) {
      geo.eta = jobj["eta"].number_value();
    } else {
      geo.eta = 1.0;
    }
    scene.objects.push_back(std::move(geo));
  }
  return scene;
}

// TODO: error handling
// only supports little endian
Image<double> ParsePfm(const std::string path) {
  // read meta data
  std::ifstream file(path, std::ios::in | std::ios::binary);
  std::string line;
  std::getline(file, line); // "PF"
  std::getline(file, line);
  std::stringstream sline(line);
  int w, h;
  sline >> w >> h;
  printf("%d, %d\n", w, h);
  Image<double> img(w, h);
  std::getline(file, line); // "-1.000000"

  // binary part
  int i = 0, x = 0, y = 0;
  Pixel<double> pix;
  while (!file.eof()) {
    float f;
    file.read((char *)&f, 4);
    switch (i) {
      case 0:
        pix.r = f;
        break;
      case 1:
        pix.g = f;
        break;
      case 2:
        pix.b = f;
        img.m[y][x] = pix;
        x++;
//        printf("%d %d %f %f %f\n", x, y, pix.r, pix.g, pix.b);
        if (x >= w) {
          x = 0;
          y++;
          if (y >= h) {
            goto END_READING;
          }
        }
        break;
      default:
        Logger::error("Something wrong!");
        break;
    }
    i = (i == 2) ? 0 : i + 1;
  }
END_READING:
  printf("%d %d\n", x, y);
  file.close();

  return img;
}
