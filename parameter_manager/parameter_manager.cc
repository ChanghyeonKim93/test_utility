
#include "parameter_manager.h"

ParameterManager::ParameterManager() {}

bridge::ParameterEntity ParameterManager::ConvertTo(
    const ParameterEntity& param) {
  bridge::ParameterEntity bridge_param;
  bridge_param.key = param.key;
  bridge_param.type = bridge::ParameterEntity::Type::kUnknown;
  if (!param.value.has_value()) return bridge_param;

  switch (param.type) {
    case ParameterEntity::Type::kBool: {
      if (param.value.type() == typeid(bool))
        bridge_param.bool_value = std::any_cast<bool>(param.value);
    } break;
    case ParameterEntity::Type::kInt: {
      if (param.value.type() == typeid(int))
        bridge_param.int_value = std::any_cast<int>(param.value);
    } break;
    case ParameterEntity::Type::kDouble: {
      if (param.value.type() == typeid(double))
        bridge_param.double_value = std::any_cast<double>(param.value);
    } break;
    case ParameterEntity::Type::kString: {
      if (param.value.type() == typeid(std::string))
        bridge_param.string_value = std::any_cast<std::string>(param.value);
      else if (param.value.type() == typeid(const char*))
        bridge_param.string_value =
            std::string(std::any_cast<const char*>(param.value));
    } break;
    default:
      break;
  }
  return bridge_param;
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
