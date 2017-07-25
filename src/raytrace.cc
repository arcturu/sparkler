#define _USE_MATH_DEFINES
#include <cstdio>
#include <cmath>
#include <random>
#include <complex>
#include "raytrace.h"
#include "image.h"
#include "geometry.h"
#include "stat.h"
#include "logger.h"

#ifdef _OPENMP
#include <omp.h>
#endif

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

Color gammaCorrection(const Color& color, double gamma) {
  return Color(pow(color.r, gamma), pow(color.g, gamma), pow(color.b, gamma));
}

Intersection getShadowIntersection(Scene& scene, Ray shadow_ray, double maxdist) {
  const double SHADOW_EPS = 1e-10;
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

Color getImageLighting(Scene& scene, const Ray& ray) {
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

double max(double a, double b) {
  return (a > b) ? a : b;
}

Color getIrradianceSingle(Scene& scene, const Light& l, const Intersection& it) {
  Color pix;
  Ray shadow_ray;
  shadow_ray.src = it.p;
  shadow_ray.dir = l.p - it.p;
  Intersection shadowIt = getShadowIntersection(scene, shadow_ray, shadow_ray.dir.length());
  if (shadowIt.hit) {
    pix.r = 0; pix.g = 0; pix.b = 0;
  } else {
    double dist = (l.p - it.p).length();
    Vector3d lv = (l.p - it.p).normalize();
    pix.r += l.color.r * max(it.n.dot(lv), 0) / 4.0 / M_PI / dist / dist * l.luminance;
    pix.g += l.color.g * max(it.n.dot(lv), 0) / 4.0 / M_PI / dist / dist * l.luminance;
    pix.b += l.color.b * max(it.n.dot(lv), 0) / 4.0 / M_PI / dist / dist * l.luminance;
  }
  return pix;
}

Color getIrradiance(Scene& scene, const Intersection& it) {
  Color pix;
  for (const auto& l : scene.lights) {
    Ray shadow_ray;
    shadow_ray.src = it.p;
    shadow_ray.dir = l.p - it.p;
    Intersection shadowIt = getShadowIntersection(scene, shadow_ray, shadow_ray.dir.length());
    if (shadowIt.hit) {
      pix.r = 0; pix.g = 0; pix.b = 0;
    } else {
      double dist = (l.p - it.p).length();
      Vector3d lv = (l.p - it.p).normalize();
      pix.r += l.color.r * max(it.n.dot(lv), 0) / 4.0 / M_PI / dist / dist * l.luminance;
      pix.g += l.color.g * max(it.n.dot(lv), 0) / 4.0 / M_PI / dist / dist * l.luminance;
      pix.b += l.color.b * max(it.n.dot(lv), 0) / 4.0 / M_PI / dist / dist * l.luminance;
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

Color getGlossy(Scene& scene, const Ray& ray, const Intersection& it) {
  Color pix;
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
    double glossy = fmax(0, pow(it.n.dot(h.normalize()), g)) * 50;
    pix.r += glossy;
    pix.g += glossy;
    pix.b += glossy;
  }
  return pix;
}

Color KajiyaKay(Scene& scene, const Ray& ray, const Intersection& it, double r) {
  Color diffuse;
  Color specular;

  // Kajiya-Kay Model
  for (const auto& light : scene.lights) {
    Color irr = getIrradianceSingle(scene, light, it);
    // Diffuse
    double d = 1 - pow(it.tan.dot((light.p - it.p).normalize()), 2);
    if (d > 0) {
      diffuse = diffuse + sqrt(d) * irr;
    }

    // Specular
    double cos_tl = it.tan.dot((light.p - it.p).normalize());
    double cos_tv = it.tan.dot(-ray.dir);
    double sin_tl = (cos_tl * cos_tl < 1) ? sqrt(1 - cos_tl * cos_tl) : 0;
    double sin_tv = (cos_tv * cos_tv < 1) ? sqrt(1 - cos_tv * cos_tv) : 0;
    double s = sin_tl * sin_tv - cos_tl * cos_tv;
    if (s > 0) {
      specular = specular + pow(s, r) * irr;
    }
  }

//    printf("%f %f\n", diffuse.r, specular.r);
  return (diffuse + specular) * it.color;
}

class CyclicAngle {
 private:
  double r; // in radian
 public:
  CyclicAngle() : r(0) {}
  CyclicAngle(double r) : r(r) {} // arg must be in radian
  static double normalize(double rad);
  double getRadian() const { return normalize(r); }
  double getDegree() const { return normalize(r) * 180 / M_PI; }
};

// valid range: [0, 2pi)
double CyclicAngle::normalize(double rad) {
  if (rad < 0) {
    double normalized = rad + 2 * M_PI;
    if (normalized >= 2 * M_PI) { // for safety (could be removed)
      return 0;
    } else {
      return CyclicAngle::normalize(rad + 2 * M_PI);
    }
  } else if (rad >= 2 * M_PI) {
    double normalized = rad - 2 * M_PI;
    if (normalized < 0) { // for safety (could be removed)
      return 0;
    } else {
      return CyclicAngle::normalize(normalized);
    }
  } else {
    return rad;
  }
}

CyclicAngle operator-(const CyclicAngle a) {
  return CyclicAngle(-a.getRadian());
}

CyclicAngle operator+(const CyclicAngle lhs, const CyclicAngle rhs) {
  return CyclicAngle(lhs.getRadian() + rhs.getRadian());
}

CyclicAngle operator-(const CyclicAngle lhs, const CyclicAngle rhs) {
  return CyclicAngle(lhs.getRadian() - rhs.getRadian());
}

CyclicAngle operator*(double c, const CyclicAngle a) {
  return CyclicAngle(c * a.getRadian());
}

CyclicAngle operator/(const CyclicAngle a, double c) {
  return CyclicAngle(a.getRadian() / c);
}

double gaussian(double s, double x) {
  return exp(-x * x / (2 * s * s)) / sqrt(2 * M_PI * s * s);
}

double fresnel(double eta, double eta_s, double eta_p, double r_i) {
  double r_t = asin(sin(r_i) / eta);

  double fs = (cos(r_i) - eta_s * cos(r_t)) / (cos(r_i) + eta_s * cos(r_t));
  double fp = (eta_p * cos(r_i) - cos(r_t)) / (eta_p * cos(r_i) + cos(r_t));
  return (fs * fs + fp * fp) / 2;
}

double dpdh(int p, double r_i, double eta_s) {
  double h = sin(r_i);
  double c = asin(1 / eta_s);
  return (6 * p * c / M_PI - 2) - 3 * 8 * p * c / pow(M_PI, 3) * r_i * r_i / sqrt(1 - h * h);
}

double d2pdh2(int p, double r_i, double eta_s) {
  double c = asin(1 / eta_s);
  return (-6 * 8 * p * c * r_i / pow(M_PI, 3) / cos(r_i)
      - (6 * p * c / M_PI - 2 - 3 * 8 * p * c / pow(M_PI, 3) * pow(r_i, 2)) / sin(r_i)) / cos(r_i);
}

// ax^3 + bx^2 + cx + d = 0
// a > 0
std::vector<std::complex<double>> solveCubic(double a, double b, double c, double d) {
  std::vector<std::complex<double>> res;
  std::complex<double> w(-0.5, sqrt(3.0) / 2.0);
  b /= a;
  c /= a;
  d /= a;

  std::complex<double> p(c - b * b / 3.0);
  std::complex<double> q(d - b * c / 3.0 + 2.0 * pow(b, 3) / 27.0);
  std::complex<double> s = std::sqrt(std::pow(q / 2.0, 2.0) + std::pow(p / 3.0, 3.0));
  for (int i = 0; i < 3; i++) {
    res.push_back(std::pow(w, i) * std::pow(-q / 2.0 + s, 1.0 / 3.0)
        + std::pow(w, 3.0 - i) * std::pow(-q / 2.0 - s, 1.0 / 3.0)
        - b / 3.0);
  }
  return res;
}

double smoothStep(double a, double b, double x) {
  if (x <= a) {
    return 0;
  } else if (b <= x) {
    return 1.0;
  } else {
    return x / (b - a);
  }
}

double getMarschnerScatter(const Light& light, const Ray& ray, const Intersection& it) {
  // TODO make configurable
  const double eta           = 1.55;
  const double sigma_a       = 0.3;
  const double a             = 0.95;
  const double alpha_r       = -7.5 * 180 / M_PI;
  const double alpha_tt      = -alpha_r / 2;
  const double alpha_trt     = -alpha_r * 3 / 2;
  const double beta_r        = 10 * 180 / M_PI;
  const double beta_tt       = beta_r / 2;
  const double beta_trt      = 2 * beta_r;
  const double k_g           = 3;
  const double w_c           = 15 * 180 / M_PI;
  const double del_eta_prime = 0.3;
  const double del_h_m       = 0.5;


  Vector3d u = it.tan.normalize();
  Vector3d w = it.n.normalize();
  Vector3d v = -u.cross(w).normalize();

  Vector3d l = (light.p - it.p).normalize();
  Vector3d l_vw = (l - l.dot(u) * u).normalize();
  Vector3d e = -ray.dir.normalize();
  Vector3d e_vw = (e - e.dot(u) * u).normalize();
  double theta_i = (M_PI / 2.0 - acos(l.dot(u)));
  double theta_r = (M_PI / 2.0 - acos(e.dot(u)));
  double theta_d = (theta_r - theta_i) / 2.0;
  double theta_h = (theta_i + theta_r) / 2.0;
  CyclicAngle phi_i((l_vw.dot(w) > 0) ? acos(l_vw.dot(v)) : -acos(l_vw.dot(v)));
  CyclicAngle phi_r((e_vw.dot(w) > 0) ? acos(e_vw.dot(v)) : -acos(e_vw.dot(v)));
  CyclicAngle phi = phi_r - phi_i;
  CyclicAngle phi_h = (phi_i + phi_r) / 2.0;

  double tmp = sqrt(pow(eta, 2.0) - pow(sin(theta_d), 2.0));
  double eta_s = tmp / cos(theta_d);
  double eta_p = pow(eta, 2.0) * cos(theta_d) / tmp;

  const double EPS = 10e-5;
  double n_r, n_tt, n_trt;
  {
    // calculate n_r
    double r_i = -phi.getRadian() / 2.0;
    n_r = fresnel(eta, eta_s, eta_p, r_i) / std::abs(2.0 * dpdh(0, r_i, eta_s));
  }
  {
    // calculate n_tt
    double c = asin(1.0 / eta_s);
//    std::vector<std::complex<double>> r_is = solveCubic(1, -2, -11, 12);
    std::vector<std::complex<double>> r_is = solveCubic(-8.0 * c / pow(M_PI, 3.0), 0, 6.0 * c / M_PI, M_PI);
//    printf("---\n");
    int real_count = 0;
    double r_i_real = 0;
    for (const auto& r_i : r_is) {
//      printf("%f %f\n", r_i.real(), r_i.imag());
      if (std::abs(r_i.imag()) < EPS) {
        r_i_real = r_i.real();
        real_count++;
      }
    }
    if (real_count > 1) {
      Logger::warn("Too many real solutions!");
//      THROW_EXCEPTION("Too many real solution!");
    }
    double r_t = asin(sin(r_i_real) / eta);
    n_tt = pow(1.0 - fresnel(eta, eta_s, eta_p, r_i_real), 2.0)
     * exp(-2.0 * sigma_a * (1.0 + cos(2 * r_t)))
     / std::abs(2.0 * dpdh(0, r_i_real, eta_s));
  }
  {
    // calculate n_trt
    double eta_star1 = 2.0 * (eta - 1) * pow(a, 2.0) - eta + 2.0;
    double eta_star2 = 2.0 * (eta - 1) * pow(a, -2.0) - eta + 2.0;
    double eta_star = (eta_star1 + eta_star2 + cos(2.0 * phi_h.getRadian()) * (eta_star1 - eta_star2)) / 2.0;
    double tmp = sqrt(pow(eta_star, 2.0) - pow(sin(theta_d), 2.0));
    double eta_s_star = tmp / cos(theta_d);
    double eta_p_star = pow(eta_star, 2.0) * cos(theta_d) / tmp;

    CyclicAngle phi_c(0);
    double del_h = del_h_m;
    double t = smoothStep(2.0, 2.0 + del_eta_prime, eta_s_star);
    if (eta_s_star < 2) {
      double h_c = sqrt((4.0 - pow(eta_s_star, 2.0)) / 3.0);
      double r_i = asin(h_c);
      double r_t = asin(h_c / eta_s_star);
      phi_c = CyclicAngle(4.0 * r_t - 2.0 * r_i + 2.0 * M_PI);
      del_h = std::min(del_h_m, 2.0 * sqrt(2.0 * w_c * abs(d2pdh2(2, r_i, eta_s_star))));
      t = 1.0;
    }
    double c = asin(1.0 / eta_s_star);
    std::vector<std::complex<double>> r_is = solveCubic(-8.0 * 2.0 * c / pow(M_PI, 3.0), 0, 6.0 * 2.0 * c / M_PI, 2.0 * M_PI);
    n_trt = 0;
    for (const auto& r_i : r_is) {
      if (r_i.imag() < EPS) {
//      printf("%f %f\n", r_i.real(), r_i.imag());
        double r_t = asin(sin(r_i.real()) / eta_s_star);
        n_trt += pow(1.0 - fresnel(eta_star, eta_s_star, eta_p_star, r_i.real()), 2.0)
          * fresnel(eta_star, 1.0 / eta_s_star, 1.0 / eta_p_star, r_t)
          * pow(exp(-2.0 * sigma_a * (1.0 + cos(2 * r_t))), 2.0)
          / abs(2 * dpdh(2, sin(r_i.real()), eta_s_star));
      }
    }
//    n_trt *= (1.0 - t * gaussian(w_c, (phi - phi_c).getRadian())) / gaussian(w_c, 0);
//    n_trt *= (1.0 - t * gaussian(w_c, (phi + phi_c).getRadian())) / gaussian(w_c, 0);
//    n_trt += t * k_g * 
  }
  double scatter = (gaussian(beta_r, theta_h - alpha_r) * n_r
    + gaussian(beta_tt, theta_h - alpha_tt) * n_tt
    + gaussian(beta_trt, theta_h - alpha_trt) * n_trt) / pow(cos(theta_d), 2.0);
//  printf("%f %f %f %f %f\n", n_r, n_tt, n_trt, cos(theta_d), scatter);
  return scatter;
}

// TODO: support other types of lights
Color shade(Scene& scene, const Ray& ray, const Intersection& it, unsigned int ttl) {
  Color pix;
  if (ttl < 1) {
//    pix.r = 255; pix.g = 0; pix.b = 0; // for debug
    pix.r = 0; pix.g = 0; pix.b = 0;
    return pix;
  }
  if (!it.hit) {
    if (scene.camera.bgImage().height() > 0) {
      return getImageLighting(scene, ray);
    } else {
      pix.r = scene.camera.bgColor().r;
      pix.g = scene.camera.bgColor().g;
      pix.b = scene.camera.bgColor().b;
    }
    return pix;
  }
  if (it.material == Diffuse) {
    double k_d = 1.0;
    pix = k_d / M_PI * getIrradiance(scene, it) * it.color;
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
    pix = getIrradiance(scene, it) + getGlossy(scene, ray, it);
  } else if (it.material == Hair) {
//    {
//      pix = KajiyaKay(scene, ray, it, 80);
//    }
    {
      pix = Color();
      for (const auto& light : scene.lights) {
        Color tmp = getMarschnerScatter(light, ray, it) * getIrradianceSingle(scene, light, it) * it.color;
  //      printf("%f %f %f\n", tmp.r, tmp.g, tmp.b);
        // TODO super ad-hoc constant is used
        pix = pix + 12 * tmp;
      }
    }
//    {
//      double k_d = M_PI;
//      pix = k_d / M_PI * getIrradiance(scene, it) * it.color;
//    }
  } else {
    Logger::error(std::string("Unknown material: ") + std::to_string(it.material));
  }

//  printf("%f %f %f\n", pix.r, pix.g, pix.b);
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
  const int N = 1;
  Image<uint8_t> img(scene.camera.film.xpixels(), scene.camera.film.ypixels());

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int y = 0; y < img.height(); y++) {
    printf("%d\n", y);
    for (int x = 0; x < img.width(); x++) {
      Pixel<double> pix;
      for (int i = 0; i < N; i++) {
        Ray ray = generateCameraRay(scene.camera, x, y);
//        Ray ray = generateCameraRayLens(scene.camera, x, y);
        stat_num_ray++;

        Intersection it = scene.intersect(ray);
        Color tmp_pix = shade(scene, ray, it, 100);
        pix.r += tmp_pix.r / N * 255;
        pix.g += tmp_pix.g / N * 255;
        pix.b += tmp_pix.b / N * 255;
      }
      img.m[y][x] = saturate(toneMap(pix), 255);
    }
  }

  return img;
}
