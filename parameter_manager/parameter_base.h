#ifndef PARAMETER_BASE_H_
#define PARAMETER_BASE_H_

#include <any>
#include <string>
#include <unordered_map>

#include "footprint.h"

class ParameterBase {
 public:
  enum class Type { kUnknown, kBool, kInt, kDouble, kString, kFootprint };

  ParameterBase() {}

  ParameterBase(const std::string& key, const std::any& value)
      : key_(key), value_(value) {
    if (!value.has_value()) throw std::runtime_error("no value");
    auto it = type_map_.find(value.type().name());
    if (it == type_map_.end())
      throw std::runtime_error("No type exists in map.");
    type_ = it->second;
  }

  ParameterBase& operator=(const ParameterBase& rhs) {
    key_ = rhs.key_;
    type_ = rhs.type_;
    value_ = rhs.value_;
    return *this;
  }

  template <typename T>
  void SetValue(const T& value) {
    if (!value_.has_value())
      throw std::runtime_error("Cannot set without value initialization");
    if (value_.type() != typeid(T)) throw std::runtime_error("Wrong type");
    value_ = value;
  }

  const std::string& GetKey() const { return key_; }

  const Type& GetType() const { return type_; }

  template <typename T>
  T GetValue() const {
    if (!value_.has_value()) throw std::runtime_error("No value");
    if (value_.type() != typeid(T))
      throw std::runtime_error(
          "Wrong type is queried. key: \"" + key_ + "\", query type: \"" +
          typeid(T).name() + "\", actual type: \"" + value_.type().name() +
          "\" in " + std::string(__FILE__) + ":" + std::to_string(__LINE__));
    return std::any_cast<T>(value_);
  }
  std::any GetValue() const {
    if (!value_.has_value()) throw std::runtime_error("No value");
    return value_;
  }

  template <typename Dst>
  Dst ConvertTo() {
    Dst dst;
    dst.key_ = key_;
    dst.type_ = type_;
    dst.value_ = value_;
    return dst;
  }

 private:
  std::string key_{""};
  Type type_{Type::kUnknown};
  std::any value_;

  const std::unordered_map<std::string, Type> type_map_{
      {typeid(bool).name(), Type::kBool},
      {typeid(int).name(), Type::kInt},
      {typeid(double).name(), Type::kDouble},
      {typeid(std::string).name(), Type::kString},
      {typeid(const char*).name(), Type::kString},
      {typeid(Footprint).name(), Type::kFootprint}};
};

#endif  // PARAMETER_BASE_H_