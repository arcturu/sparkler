#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include "raytrace.h"
#include "image.h"
#include "geometry.h"

Ray generateCameraRay(Camera cam, int x, int y) {
  Ray r;
  r.src = (cam.film.res * (y + 0.5) - cam.film.h / 2.0) * cam.u()
    + (cam.film.res * (x + 0.5) - cam.film.w / 2.0) * cam.v()
    + cam.film.z * cam.w()
    + cam.p();
  r.dir = (r.src - cam.p()).normalize();
  return r;
}

uint8_t saturate(double x, uint8_t limit) {
  if (x > (double)limit) {
    return limit;
  } else if (x < 0) {
    return 0;
  } else {
    return (uint8_t)x;
  }
}

Pixel<uint8_t> shade(Intersection it) {
  Vector3d l(5, 5, 5);
  double luminance = 20000.0;
  Pixel<uint8_t> pix;
  pix.r = pix.g = pix.b = saturate(it.n.dot(l - it.p) / 4.0 / M_PI / std::pow((it.p - l).length(), 2) * luminance, 255);
  return pix;
}

Image<uint8_t> raytrace(Camera cam, Geometry geo) {
  Image<uint8_t> img(cam.film.w / cam.film.res, cam.film.h / cam.film.res);

  for (int y = 0; y < img.height(); y++) {
    for (int x = 0; x < img.width(); x++) {
      Ray r = generateCameraRay(cam, x, y);

      Intersection it = geo.intersect(r);
      if (it.hit) {
//        img.m[y][x].r = it.n.x * 255;
//        img.m[y][x].g = it.n.y * 255;
//        img.m[y][x].b = it.n.z * 255;
        img.m[y][x] = shade(it);
      }
    }
  }

  return img;
}
