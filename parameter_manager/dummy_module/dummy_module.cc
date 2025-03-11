#include "dummy_module/dummy_module.h"

namespace dummy_module {

DummyModule::DummyModule() {}

void DummyModule::LoadParametersFromYaml() {
  ParameterEntity param_entity;
  param_entity.key = "foot_print.length";
  param_entity.type = ParameterEntity::Type::kDouble;
  param_entity.value = 1.0;
  default_parameter_table_.insert({param_entity.key, param_entity});
  param_entity.key = "foot_print.width";
  param_entity.type = ParameterEntity::Type::kDouble;
  param_entity.value = 1.0;
  default_parameter_table_.insert({param_entity.key, param_entity});
  param_entity.key = "max_linear_velocity";
  param_entity.type = ParameterEntity::Type::kDouble;
  param_entity.value = 1.0;
  default_parameter_table_.insert({param_entity.key, param_entity});
}

bool DummyModule::SetParameters(
    const std::vector<bridge::ParameterEntity>& bridge_parameter_entity_list) {
  if (bridge_parameter_entity_list.empty()) return true;  // empty -> success

  // Parameter buffer
  std::vector<ParameterEntity> buffer_parameter_entity_list;
  for (const auto& bridge_param : bridge_parameter_entity_list) {
    if (!IsValid(bridge_param)) return false;

    // Set!
  }

  return true;
}

bool DummyModule::ResetParameters(
    const std::vector<std::string>& parameter_key_list) {}

std::vector<bridge::ParameterEntity> DummyModule::GetCurrentParameters(
    const std::vector<std::string>& parameter_key_list) {}

std::vector<bridge::ParameterEntity> DummyModule::GetSupportedParameterList() {}

bool DummyModule::IsValid(
    const bridge::ParameterEntity& bridge_parameter_entity) {
  if (default_parameter_table_.find(bridge_parameter_entity.key) ==
      default_parameter_table_.end())
    return false;

  return true;
}

}  // namespace dummy_module