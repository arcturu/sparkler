#include <iostream>
#include "logger.h"

void Logger::info(const std::string& msg) {
  std::cout << "\x1b[36m[info]\x1b[39m " << msg << std::endl;
}

void Logger::warn(const std::string& msg) {
  std::cout << "\x1b[33m[warning] " << msg << "\x1b[39m" << std::endl;
}

void Logger::error(const std::string& msg) {
  std::cout << "\x1b[31m[error] " << msg << "\x1b[39m" << std::endl;
}
