#ifndef OPTIMIZER_COUT_UTILITY_H_
#define OPTIMIZER_COUT_UTILITY_H_

#include <string>

#define TEXT_RED(str) (std::string("\033[0;31m") + str + std::string("\033[0m"))
#define TEXT_GREEN(str) \
  (std::string("\033[0;32m") + str + std::string("\033[0m"))
#define TEXT_YELLOW(str) \
  (std::string("\033[0;33m") + str + std::string("\033[0m"))
#define TEXT_BLUE(str) \
  (std::string("\033[0;34m") + str + std::string("\033[0m"))
#define TEXT_MAGENTA(str) \
  (std::string("\033[0;35m") + str + std::string("\033[0m"))
#define TEXT_CYAN(str) \
  (std::string("\033[0;36m") + str + std::string("\033[0m"))

#endif  // OPTIMIZER_COUT_UTILITY_H_