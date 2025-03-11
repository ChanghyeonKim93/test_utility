#ifndef PARAMETER_MANAGER_BRIDGE_H_
#define PARAMETER_MANAGER_BRIDGE_H_

#include <optional>
#include <string>

namespace bridge {

struct ParameterEntity {
  enum class Type { kUnknown = -1, kBool, kInt, kDouble, kString };
  std::string key{""};
  Type type{Type::kUnknown};
  std::optional<bool> bool_value{std::nullopt};
  std::optional<int> int_value{std::nullopt};
  std::optional<double> double_value{std::nullopt};
  std::optional<std::string> string_value{std::nullopt};
};

}  // namespace bridge

#endif  // PARAMETER_MANAGER_BRIDGE_H_