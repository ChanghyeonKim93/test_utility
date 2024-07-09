#include <iostream>

#include "parameter_base.h"

namespace motion_planner {
namespace forward_planner {

struct Parameter : public ParameterBase {
 public:
 private:
};

}  // namespace forward_planner
}  // namespace motion_planner

namespace motion_planner {
namespace forward_planner {
namespace bridge {

struct Parameter : public ParameterBase {
 public:
 private:
};

}  // namespace bridge
}  // namespace forward_planner
}  // namespace motion_planner

namespace behavior_planner {
namespace bridge {

struct Parameter : public ParameterBase {
 public:
 private:
};

}  // namespace bridge
}  // namespace behavior_planner

namespace behavior_planner {

struct Parameter : public ParameterBase {
 public:
 private:
};

}  // namespace behavior_planner

int main() {
  // bp bridge -> bp -> mp bridge -> mp
  motion_planner::forward_planner::bridge::Parameter bridge_param;
  motion_planner::forward_planner::Parameter param;

  param = bridge_param.ConvertTo<motion_planner::forward_planner::Parameter>();

  return 0;
}