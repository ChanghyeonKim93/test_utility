#ifndef PARAMETER_BASE_H_
#define PARAMETER_BASE_H_

#include <any>
#include <string>

class ParameterBase {
 public:
  enum class Type { kUnknown = -1, kBool, kInt, kDouble, kString, kFootprint };

  const std::string& GetKey() const { return key_; }

  Type GetType() const { return type_; }

  template <typename T>
  T GetValue() const {
    return std::any_cast<T>(value_);
  }

  template <typename TargetType>
  TargetType ConvertTo() const {
    return {};
  }

 private:
  bool CheckValidType();

  std::string key_{""};
  Type type_{Type::kUnknown};
  std::any value_;
};

#endif  // PARAMETER_BASE_H_