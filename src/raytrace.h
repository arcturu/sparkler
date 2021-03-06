#ifndef RAYTRACE_H_
#define RAYTRACE_H_

#include "image.h"
#include "geometry.h"

Image<uint8_t> raytrace(Scene& scene);

#endif
