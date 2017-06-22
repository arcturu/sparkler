#ifndef PARSER_H_
#define PARSER_H_

#include <string>
#include "geometry.h"
#include "image.h"

Geometry ParseObj(const std::string path); // parsing .obj files
Scene ParseScene(const char *path, const char *obj_dir); // parse scene json
Camera ParseObjxCamera(const char *path, bool *st);
Image<double> ParsePfm(const std::string path); // parse .pfm HDR image
#endif
