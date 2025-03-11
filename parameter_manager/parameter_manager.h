#ifndef PARAMETER_MANAGER_PARAMETER_MANAGER_H_
#define PARAMETER_MANAGER_PARAMETER_MANAGER_H_

#include <map>
#include <set>

#include "bridge.h"
#include "types.h"

class ParameterManager {
 public:
  ParameterManager();

 private:
  bridge::ParameterEntity ConvertTo(const ParameterEntity& parameter);
  ParameterEntity ConvertTo(const bridge::ParameterEntity& bridge_parameter);

  std::map<std::string, ParameterEntity> pa_;
};

#endif  // PARAMETER_MANAGER_PARAMETER_MANAGER_H_