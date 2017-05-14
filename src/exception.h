#ifndef EXCEPTION_H_
#define EXCEPTION_H_

#include <exception>

#define THROW_EXCEPTION(message) \
  throw Exception(message, __FILE__, __func__, __LINE__);

class Exception : std::exception {
 private:
  std::string message;
  const char *file;
  const char *func;
  int line;
 public:
  Exception(const std::string& aMessage, const char *aFile, const char *aFunc, int aLine) : message(aMessage), file(aFile), func(aFunc), line(aLine) {}
  const char *what() const throw() {
    return (message + "\nFile: " + file + "\nFunc: " + func + "\nLine: " + std::to_string(line)).c_str();
  }
};

#endif
