#define _USE_MATH_DEFINES
#include <cstdio>
#include <cmath>
#include <random>
#include "raytrace.h"
#include "image.h"
#include "geometry.h"
#include "stat.h"
#include "logger.h"

Ray generateCameraRay(const Camera& cam, int x, int y) {
  Ray r;
  r.src = (cam.film.resolution() * (y + 0.5) - cam.film.height() / 2.0) * cam.u()
    + (cam.film.resolution() * (x + 0.5) - cam.film.width() / 2.0) * cam.v()
    + cam.film.distance() * cam.w()
    + cam.p();
  r.dir = (r.src - cam.p()).normalize();
  return r;
}

Ray generateCameraRayLens(const Camera& cam, int x, int y) {
  double lens_d = 15; // TODO from json
  double lens_r = 1.5; // TODO from json
  Vector3d src = (cam.film.resolution() * (y + 0.5) - cam.film.height() / 2.0) * cam.u()
    + (cam.film.resolution() * (x + 0.5) - cam.film.width() / 2.0) * cam.v()
    - cam.film.distance() * cam.w()
    + cam.p();
  Vector3d lens_center = cam.p();
  double t = lens_d / cam.film.distance();
  Vector3d target = src + t * (lens_center - src);
//  Logger::info(target.toString());

  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::uniform_real_distribution<double> rand(0, 1);

  double lens_x, lens_y;
  do {
    lens_x = (2.0 * rand(mt) - 1) * lens_r;
    lens_y = (2.0 * rand(mt) - 1) * lens_r;
  } while ((lens_x * lens_x + lens_y * lens_y) < lens_r * lens_r);

  Ray r;
  r.src = cam.p() + lens_y * cam.u() + lens_x * cam.v() + cam.w();
  r.dir = (target - r.src).normalize();
//  Logger::info(r.src.toString() + r.dir.toString());

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

Intersection getShadowIntersection(Scene& scene, Ray shadow_ray, double maxdist) {
  const double SHADOW_EPS = 1e-13;
  Vector3d p = shadow_ray.src;
  Intersection it = scene.intersect(shadow_ray);
  while (it.hit && (it.p - p).length() < SHADOW_EPS) {
    shadow_ray.src = SHADOW_EPS * shadow_ray.dir + it.p;
    it = scene.intersect(shadow_ray);
  }
  if ((it.p - p).length() > maxdist) {
    it.hit = false;
  }
  return it;
}

// TODO: support other types of lights
Pixel<uint8_t> shade(Scene& scene, const Ray& ray, const Intersection& it, unsigned int ttl) {
  Pixel<uint8_t> pix;
  if (ttl < 1) {
    pix.r = 255; pix.g = 0; pix.b = 0;
    return pix;
  }
  if (!it.hit) {
    pix.r = scene.camera.bgColor().r * 255;
    pix.g = scene.camera.bgColor().g * 255;
    pix.b = scene.camera.bgColor().b * 255;
    return pix;
  }
  if (it.material == Diffuse) {
    for (const auto& l : scene.lights) {
      Ray shadow_ray;
      shadow_ray.src = it.p;
      shadow_ray.dir = l.p - it.p;
      Intersection shadowIt = getShadowIntersection(scene, shadow_ray, shadow_ray.dir.length());
      if (shadowIt.hit) {
        pix.r = 0; pix.g = 0; pix.b = 0;
      } else {
        double dist = (l.p - it.p).length();
        pix.r += saturate(l.color.r * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 6, 255);
        pix.g += saturate(l.color.g * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 6, 255);
        pix.b += saturate(l.color.b * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 6, 255);
      }
    }
  } else if (it.material == Mirror) {
    Ray reflected_ray;
    reflected_ray.src = it.p;
    reflected_ray.dir = ray.dir.normalize() - 2.0 * it.n.dot(ray.dir.normalize()) * it.n;
    return shade(scene, reflected_ray, getShadowIntersection(scene, reflected_ray, 1e100), ttl-1);
  } else if (it.material == Glass) { // TODO: 複数の屈折率の物質が重なるとバグる？
    double eta1 = 1.0;

    double wn = ray.dir.normalize().dot(it.n);
    double relative_eta;
    Vector3d n;
    if (wn < 0) {
      relative_eta = eta1 / it.eta;
      n = it.n;
    } else {
      relative_eta = it.eta / eta1;
      n = -it.n;
      wn = -wn;
    }
    double r = 1.0 - pow(relative_eta, 2) * (1.0 - pow(wn, 2));
    if (r < 0) {
      // Mirror
//      pix.r = 0; pix.g = 255; pix.b = 0;
//      return pix;
      Ray reflected_ray;
      reflected_ray.src = it.p;
      reflected_ray.dir = ray.dir.normalize() - 2.0 * n.dot(ray.dir.normalize()) * n;
      return shade(scene, reflected_ray, getShadowIntersection(scene, reflected_ray, 1e100), ttl-1);
    } else {
      // refraction
      Ray refracted_ray;
      refracted_ray.src = it.p;
      refracted_ray.dir = relative_eta * (ray.dir.normalize() - wn * n) - sqrt(r) * n;
      return shade(scene, refracted_ray, getShadowIntersection(scene, refracted_ray, 1e100), ttl-1);
    }
  } else if (it.material == Glossy) {
    for (const auto& l : scene.lights) {
      Ray shadow_ray;
      shadow_ray.src = it.p;
      shadow_ray.dir = l.p - it.p;
      Intersection shadowIt = getShadowIntersection(scene, shadow_ray, shadow_ray.dir.length());
      if (shadowIt.hit) {
        pix.r = 0; pix.g = 0; pix.b = 0;
      } else {
        double dist = (l.p - it.p).length();
        pix.r += saturate(l.color.r * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 6, 255);
        pix.g += saturate(l.color.g * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 6, 255);
        pix.b += saturate(l.color.b * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 6, 255);
      }

      Vector3d h = shadow_ray.dir.normalize() - ray.dir.normalize();
      double g = 5;
      double glossy = fmax(0, pow(it.n.dot(h.normalize()), g)) * 255 * 0.1;
      printf("%f\n", glossy);
      pix.r = saturate((double)pix.r + glossy, 255);
      pix.g = saturate((double)pix.g + glossy, 255);
      pix.b = saturate((double)pix.b + glossy, 255);
    }
  } else {
    Logger::error(std::string("Unknown material: ") + std::to_string(it.material));
  }

  return pix;
}

Pixel<uint8_t> toneMap(const Pixel<uint8_t>& pix) {
  Pixel<uint8_t> pix2;
  double gamma = 0.9; // TODO json
  pix2.r = saturate(pow(pix.r, gamma), 255);
  pix2.g = saturate(pow(pix.g, gamma), 255);
  pix2.b = saturate(pow(pix.b, gamma), 255);
  return pix2;
}

Image<uint8_t> raytrace(Scene& scene) {
  const int N = 5;
  Image<uint8_t> img(scene.camera.film.xpixels(), scene.camera.film.ypixels());

  for (int y = 0; y < img.height(); y++) {
    printf("%d\n", y);
    for (int x = 0; x < img.width(); x++) {
      for (int i = 0; i < N; i++) {
        Ray ray = generateCameraRay(scene.camera, x, y);
//        Ray ray = generateCameraRayLens(scene.camera, x, y);
        stat_num_ray++;

        Intersection it = scene.intersect(ray);
        Pixel<uint8_t> pix = shade(scene, ray, it, 100);
        img.m[y][x].r += (double)pix.r / N;
        img.m[y][x].g += (double)pix.g / N;
        img.m[y][x].b += (double)pix.b / N;
      }
      img.m[y][x] = toneMap(img.m[y][x]);
    }
  }

  return img;
}
