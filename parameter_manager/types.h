#ifndef PARAMETER_MANAGER_TYPES_H_
#define PARAMETER_MANAGER_TYPES_H_

#include <any>
#include <string>

struct ParameterEntity {
  enum class Type { kUnknown = -1, kBool, kInt, kDouble, kString };
  std::string key{""};
  Type type{Type::kUnknown};
  std::any value;
};

#endif  // PARAMETER_MANAGER_TYPES_H_