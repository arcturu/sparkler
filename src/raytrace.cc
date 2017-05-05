#include <iostream>
#include "raytrace.h"
#include "image.h"
#include "geometry.h"

Ray generateCameraRay(Camera cam, int x, int y) {
  Ray r;
  r.src = (cam.film.res * (y + 0.5) - cam.film.h / 2.0) * cam.u()
    + (cam.film.res * (x + 0.5) - cam.film.w / 2.0) * cam.v()
    + cam.film.z * cam.w()
    + cam.p;
  r.dir = (r.src - cam.p).normalize();
  return r;
}

Image<uint8_t> raytrace(Camera cam, Geometry geo) {
  Image<uint8_t> img(cam.film.w / cam.film.res, cam.film.h / cam.film.res);

  for (int y = 0; y < img.height(); y++) {
    for (int x = 0; x < img.width(); x++) {
      Ray r = generateCameraRay(cam, x, y);

      Intersection intersection = geo.intersect(r);
      img.m[y][x].r = intersection.n.x * 255;
      img.m[y][x].g = intersection.n.y * 255;
      img.m[y][x].b = intersection.n.z * 255;
    }
  }

  return img;
}
