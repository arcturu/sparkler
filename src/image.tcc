#ifdef IMAGE_H_

#include <fstream>

template <class T>
int Image<T>::width() {
  if (m.size() > 0) {
    return m[0].size();
  } else {
    return -1;
  }
}

template <class T>
void Image<T>::outputPpm(const char *file) {
  std::ofstream fout(file);
  fout << "P3\n";
  fout << width() << " " << height() << "\n";
  fout << "255\n";
  for (int y = height()-1; y >= 0; y--) {
    for (int x = 0; x < width(); x++) {
      fout << std::to_string(m[y][x].r) << " " << std::to_string(m[y][x].g) << " " << std::to_string(m[y][x].b) << "\n";
    }
  }
}

#endif
