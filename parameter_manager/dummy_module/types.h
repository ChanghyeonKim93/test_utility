#ifndef DUMMY_MODULE_TYPES_H_
#define DUMMY_MODULE_TYPES_H_

#include <any>
#include <string>

namespace dummy_module {

struct ParameterEntity {
  enum class Type { kUnknown = -1, kBool, kInt, kDouble, kString };
  std::string key{""};
  Type type{Type::kUnknown};
  std::any value;
};

}  // namespace dummy_module

#endif  // DUMMY_MODULE_TYPES_H_