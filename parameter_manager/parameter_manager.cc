
#include "parameter_manager.h"

ParameterManager::ParameterManager() {}

bridge::ParameterEntity ParameterManager::ConvertTo(
    const ParameterEntity& parameter) {
  bridge::ParameterEntity bridge_parameter;
  bridge_parameter.key = parameter.key;
  bridge_parameter.type = bridge::ParameterEntity::Type::kUnknown;
  if (!parameter.value.has_value()) return bridge_parameter;

  switch (parameter.type) {
    case ParameterEntity::Type::kBool: {
      if (parameter.value.type() == typeid(bool))
        bridge_parameter.bool_value = std::any_cast<bool>(parameter.value);
    } break;
    case ParameterEntity::Type::kInt: {
      if (parameter.value.type() == typeid(int))
        bridge_parameter.int_value = std::any_cast<int>(parameter.value);
    } break;
    case ParameterEntity::Type::kDouble: {
      if (parameter.value.type() == typeid(double))
        bridge_parameter.double_value = std::any_cast<double>(parameter.value);
    } break;
    case ParameterEntity::Type::kString: {
      if (parameter.value.type() == typeid(std::string)) {
        bridge_parameter.string_value =
            std::any_cast<std::string>(parameter.value);
      } else if (parameter.value.type() == typeid(const char*)) {
        bridge_parameter.string_value =
            std::string(std::any_cast<const char*>(parameter.value));
      }
    } break;
    default:
      break;
  }
  return bridge_parameter;
}

ParameterEntity ParameterManager::ConvertTo(
    const bridge::ParameterEntity& bridge_parameter) {
  ParameterEntity parameter;
  parameter.key = bridge_parameter.key;
  parameter.type = ParameterEntity::Type::kUnknown;
  switch (bridge_parameter.type) {
    case bridge::ParameterEntity::Type::kBool: {
      if (bridge_parameter.bool_value.has_value()) {
        parameter.type = ParameterEntity::Type::kBool;
        parameter.value = bridge_parameter.bool_value.value();
      }
    } break;
    case bridge::ParameterEntity::Type::kInt: {
      if (bridge_parameter.int_value.has_value()) {
        parameter.type = ParameterEntity::Type::kInt;
        parameter.value = bridge_parameter.int_value.value();
      }
    } break;
    case bridge::ParameterEntity::Type::kDouble: {
      if (bridge_parameter.double_value.has_value()) {
        parameter.type = ParameterEntity::Type::kDouble;
        parameter.value = bridge_parameter.double_value.value();
      }
    } break;
    case bridge::ParameterEntity::Type::kString: {
      if (bridge_parameter.string_value.has_value()) {
        parameter.type = ParameterEntity::Type::kString;
        parameter.value = bridge_parameter.string_value.value();
      }
    } break;
    default:
      break;
  }
  return parameter;
}
