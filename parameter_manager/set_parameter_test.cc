#include <iostream>
#include <stdexcept>

#include "parameter_base.h"

namespace motion_planner::forward_planner {
class Parameter : public ParameterBase {};
namespace bridge {

class Parameter : public ParameterBase {
 public:
  Parameter() : ParameterBase() {}
  Parameter(const std::string& key, const std::any& value)
      : ParameterBase(key, value) {}
};

}  // namespace bridge
}  // namespace motion_planner::forward_planner

namespace behavior_planner {
class Parameter : public ParameterBase {
 public:
  Parameter() : ParameterBase() {}
  Parameter(const std::string& key, const std::any& value)
      : ParameterBase(key, value) {}
};
namespace bridge {
class Parameter : public ParameterBase {
 public:
  Parameter() : ParameterBase() {}
  Parameter(const std::string& key, const std::any& value)
      : ParameterBase(key, value) {}
};
}  // namespace bridge
}  // namespace behavior_planner

int main() {
  // bp bridge -> bp -> mp bridge -> mp
  try {
    using namespace motion_planner;
    forward_planner::bridge::Parameter fp_bridge_param = {"max_linear_velocity",
                                                          1};

    behavior_planner::bridge::Parameter bp_bridge_param =
        fp_bridge_param.ConvertTo<behavior_planner::bridge::Parameter>();
    behavior_planner::Parameter bp_param;
    bp_bridge_param =
        fp_bridge_param.ConvertTo<behavior_planner::bridge::Parameter>();
    bp_param = bp_bridge_param.ConvertTo<behavior_planner::Parameter>();

    std::cerr << bp_param.GetKey() << std::endl;
    std::cerr << static_cast<int>(bp_param.GetType()) << std::endl;
    std::cerr << bp_param.GetValue<int>() << std::endl;

    std::vector<forward_planner::bridge::Parameter> fp_bridge_param_list;
    fp_bridge_param_list.push_back({"a", "e"});
  } catch (std::exception& e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
  }
  return 0;
}