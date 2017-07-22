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
  double lens_d = 7; // TODO from json
  double lens_r = 0.1; // TODO from json
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

Vector3d getRandomVector() {
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::uniform_real_distribution<double> rand(0, 1);
  Vector3d v(rand(mt), rand(mt), rand(mt));
  return v.normalize();
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

Pixel<uint8_t> saturate(Pixel<double> pix, uint8_t limit) {
  Pixel<uint8_t> pix2;
  if (pix.r > (double)limit) {
    pix2.r = limit;
  } else if (pix.r < 0) {
    pix2.r = 0;
  } else {
    pix2.r = (uint8_t)pix.r;
  }

  if (pix.g > (double)limit) {
    pix2.g = limit;
  } else if (pix.g < 0) {
    pix2.g = 0;
  } else {
    pix2.g = (uint8_t)pix.g;
  }

  if (pix.b > (double)limit) {
    pix2.b = limit;
  } else if (pix.b < 0) {
    pix2.b = 0;
  } else {
    pix2.b = (uint8_t)pix.b;
  }
  return pix2;
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

Pixel<double> getImageLighting(Scene& scene, const Ray& ray) {
  Vector3d center(0, 0, 1);
  double t = ray.dir.normalize().dot(center);
  Vector3d dir = ray.dir.normalize() - center;
  int x = (0.5 + t * dir.x() / 2) * scene.camera.bgImage().width();
  int y = (0.5 + t * dir.y() / 2) * scene.camera.bgImage().height();
  if (x < 0) {
    printf("%d %d\n", x, y);
    Logger::info(ray.dir.toString() + dir.toString());
  }
  return 255 * scene.camera.bgImage().m[y][x];
}

Pixel<double> getIrradiance(Scene& scene, const Intersection& it) {
  Pixel<double> pix;
  for (const auto& l : scene.lights) {
    Ray shadow_ray;
    shadow_ray.src = it.p;
    shadow_ray.dir = l.p - it.p;
    Intersection shadowIt = getShadowIntersection(scene, shadow_ray, shadow_ray.dir.length());
    if (shadowIt.hit) {
      pix.r = 0; pix.g = 0; pix.b = 0;
    } else {
      double dist = (l.p - it.p).length();
      pix.r += l.color.r * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 6;
      pix.g += l.color.g * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 6;
      pix.b += l.color.b * it.n.dot(l.p - it.p) / 4.0 / M_PI / dist / dist * l.luminance * 6;
    }
  }
  // image based lighting
//  if (scene.camera.bgImage().height() > 0) {
//    const int N = 10;
//    int success_count = 0;
//    Pixel<double> pix_ibl;
//    for (int i = 0; i < N; i++) {
//      Ray ray;
//      ray.src = it.p;
//      ray.dir = getRandomVector();
//      Intersection img_it = getShadowIntersection(scene, ray, 1e100);
//      if (!img_it.hit) {
//        pix_ibl = pix_ibl + 0.3 * getImageLighting(scene, ray);
//        success_count += 1;
//      }
//    }
//    pix = pix + pix_ibl / success_count;
//  }
  return pix;
}

Pixel<double> getGlossy(Scene& scene, const Intersection& it, const Ray& ray) {
  Pixel<double> pix;
  for (const auto& l : scene.lights) {
    Ray shadow_ray;
    shadow_ray.src = it.p;
    shadow_ray.dir = l.p - it.p;
    Intersection shadow_it = getShadowIntersection(scene, shadow_ray, shadow_ray.dir.length());
    if (shadow_it.hit) {
      continue;
    }
    Vector3d h = shadow_ray.dir.normalize() - ray.dir.normalize();
    double g = 40;
    double glossy = fmax(0, pow(it.n.dot(h.normalize()), g)) * 255 * 50;
    pix.r += glossy;
    pix.g += glossy;
    pix.b += glossy;
  }
  return pix;
}

// TODO: support other types of lights
Pixel<double> shade(Scene& scene, const Ray& ray, const Intersection& it, unsigned int ttl) {
  Pixel<double> pix;
  if (ttl < 1) {
//    pix.r = 255; pix.g = 0; pix.b = 0; // for debug
    pix.r = 0; pix.g = 0; pix.b = 0;
    return pix;
  }
  if (!it.hit) {
    if (scene.camera.bgImage().height() > 0) {
      return getImageLighting(scene, ray);
    } else {
      pix.r = scene.camera.bgColor().r * 255;
      pix.g = scene.camera.bgColor().g * 255;
      pix.b = scene.camera.bgColor().b * 255;
    }
    return pix;
  }
  if (it.material == Diffuse) {
    pix = getIrradiance(scene, it);
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
      if (reflected_ray.dir.x() != reflected_ray.dir.x()) {
        printf("pohe\n");
        Logger::info(ray.dir.toString() + n.toString());
      }
      return shade(scene, reflected_ray, getShadowIntersection(scene, reflected_ray, 1e100), ttl-1);
    } else {
      // refraction
      Ray refracted_ray;
      refracted_ray.src = it.p;
      refracted_ray.dir = relative_eta * (ray.dir.normalize() - wn * n) - sqrt(r) * n;
      if (refracted_ray.dir.x() != refracted_ray.dir.x()) {
        printf("hoge\n");
        Logger::info(ray.dir.toString() + n.toString());
      }
      return shade(scene, refracted_ray, getShadowIntersection(scene, refracted_ray, 1e100), ttl-1);
    }
  } else if (it.material == Glossy) {
    pix = getIrradiance(scene, it) + getGlossy(scene, it, ray);
  } else {
    Logger::error(std::string("Unknown material: ") + std::to_string(it.material));
  }

  return pix;
}

Pixel<double> toneMap(const Pixel<double>& pix) {
  Pixel<double> pix2;
  double c = 0.01;
  pix2.r = (1.0 - exp(-c * pix.r)) * 255;
  pix2.g = (1.0 - exp(-c * pix.g)) * 255;
  pix2.b = (1.0 - exp(-c * pix.b)) * 255;
/*
  double gamma = 0.9; // TODO json
  pix2.r = pow(pix.r, gamma);
  pix2.g = pow(pix.g, gamma);
  pix2.b = pow(pix.b, gamma);
*/
  return pix2;
}

Image<uint8_t> raytrace(Scene& scene) {
  const int N = 10;
  Image<uint8_t> img(scene.camera.film.xpixels(), scene.camera.film.ypixels());

  for (int y = 0; y < img.height(); y++) {
    printf("%d\n", y);
    for (int x = 0; x < img.width(); x++) {
      Pixel<double> pix;
      for (int i = 0; i < N; i++) {
        Ray ray = generateCameraRay(scene.camera, x, y);
//        Ray ray = generateCameraRayLens(scene.camera, x, y);
        stat_num_ray++;

        Intersection it = scene.intersect(ray);
        Pixel<double> tmp_pix = shade(scene, ray, it, 100);
        pix.r += tmp_pix.r / N;
        pix.g += tmp_pix.g / N;
        pix.b += tmp_pix.b / N;
      }
//      printf ("%d %d %f %f %f\n", x, y, pix.r, pix.g, pix.b);
//      img.m[img.height() - y - 1][img.width() - x - 1] = saturate(toneMap(pix), 255);
      img.m[y][x] = saturate(toneMap(pix), 255);
    }
  }

  return img;
}
