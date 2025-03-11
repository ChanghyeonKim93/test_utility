#ifndef DUMMY_MODULE_DUMMY_MODULE_H_
#define DUMMY_MODULE_DUMMY_MODULE_H_

#include <map>
#include <set>
#include <string>
#include <vector>

#include "dummy_module/bridge.h"
#include "dummy_module/types.h"

namespace dummy_module {

class DummyModule {
 public:
  DummyModule();

  bool SetParameters(
      const std::vector<bridge::ParameterEntity>& bridge_parameter_entity_list);

  bool ResetParameters(const std::vector<std::string>& parameter_key_list);

  std::vector<bridge::ParameterEntity> GetCurrentParameters(
      const std::vector<std::string>& parameter_key_list);

  std::vector<bridge::ParameterEntity> GetSupportedParameterList();

 private:
  void LoadParametersFromYaml();
  bool IsValid(const bridge::ParameterEntity& bridge_parameter_entity);

  std::map<std::string, ParameterEntity> default_parameter_table_;
  std::map<std::string, ParameterEntity> dynamic_parameter_table_;
};

}  // namespace dummy_module

#endif  // DUMMY_MODULE_DUMMY_MODULE_H_