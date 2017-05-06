#ifndef PARSER_H_
#define PARSER_H_

#include "geometry.h"

Geometry ParseObj(const char *path); // parsing .obj files
Camera ParseObjxCamera(const char *path, bool *st);

#endif
