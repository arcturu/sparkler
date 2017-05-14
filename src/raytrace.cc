#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include "raytrace.h"
#include "image.h"
#include "geometry.h"

Ray generateCameraRay(const Camera& cam, int x, int y) {
  Ray r;
  r.src = (cam.film.resolution() * (y + 0.5) - cam.film.height() / 2.0) * cam.u()
    + (cam.film.resolution() * (x + 0.5) - cam.film.width() / 2.0) * cam.v()
    + cam.film.distance() * cam.w()
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

// TODO: support other types of lights
Pixel<uint8_t> shade(Intersection it, std::vector<Light> ls) {
  Pixel<uint8_t> pix;
  for (const auto& l : ls) {
    double dist = (l.p - it.p).length();
    pix.r += saturate(l.color.r * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 10, 255);
    pix.g += saturate(l.color.g * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 10, 255);
    pix.b += saturate(l.color.b * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 10, 255);
  }
  return pix;
}

Image<uint8_t> raytrace(Scene& scene) {
  Image<uint8_t> img(scene.camera.film.xpixels(), scene.camera.film.ypixels());

  for (int y = 0; y < img.height(); y++) {
    for (int x = 0; x < img.width(); x++) {
      Ray r = generateCameraRay(scene.camera, x, y);

      Intersection it = scene.intersect(r);
      if (it.hit) {
//        img.m[y][x].r = it.n.x() * 255; img.m[y][x].g = it.n.y() * 255; img.m[y][x].b = it.n.z() * 255;
        img.m[y][x] = shade(it, scene.lights);
      }
    }
  }

  return img;
}
