#ifndef UTILITY_STRING_MODIFIER_H_
#define UTILITY_STRING_MODIFIER_H_

#include <string>

namespace utility {

class StringModifier {
 public:
  static std::string MakeRed(const std::string& str) {
    return {"\033[0;31m" + str + "\033[0m"};
  }

  static std::string MakeGreen(const std::string& str) {
    return {"\033[0;32m" + str + "\033[0m"};
  }

  static std::string MakeYellow(const std::string& str) {
    return {"\033[0;33m" + str + "\033[0m"};
  }

  static std::string MakeBlue(const std::string& str) {
    return {"\033[0;34m" + str + "\033[0m"};
  }

  static std::string MakeMagenta(const std::string& str) {
    return {"\033[0;35m" + str + "\033[0m"};
  }

  static std::string MakeCyan(const std::string& str) {
    return {"\033[0;36m" + str + "\033[0m"};
  }
};

}  // namespace utility

#endif  // UTILITY_STRING_MODIFIER_H_