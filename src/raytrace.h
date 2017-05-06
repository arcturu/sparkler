#ifndef RAYTRACE_H_
#define RAYTRACE_H_

#include "image.h"
#include "geometry.h"
#include "accel.h"

Image<uint8_t> raytrace(Camera cam, AccelStructure& accel);

#endif
