#ifndef DUMMY_MODULE_BRIDGE_H_
#define DUMMY_MODULE_BRIDGE_H_

#include <optional>
#include <string>

namespace dummy_module {
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
}  // namespace dummy_module

#endif  // DUMMY_MODULE_BRIDGE_H_
