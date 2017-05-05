#ifndef IMAGE_H_
#define IMAGE_H_

#include <vector>

template <class T>
class Pixel {
 public:
  T r;
  T g;
  T b;
};

template <class T>
class Image {
 public:
  std::vector<std::vector<Pixel<T>>> m;

  Image(int w, int h) {
    m.resize(h);
    for (auto row = m.begin(); row != m.end(); ++row) {
      row->resize(w);
    }
  }
  int width();
  int height() { return m.size(); }
  void outputPpm(const char *file);
};

#include "image.tcc"

#endif
