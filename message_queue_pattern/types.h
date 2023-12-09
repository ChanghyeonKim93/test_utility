#ifndef MESSAGE_QUEUE_PATTERN_TYPES_H_
#define MESSAGE_QUEUE_PATTERN_TYPES_H_
#include <vector>

struct Point {
  double x{0.0};
  double y{0.0};
};

struct Pose {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct Scan {
  int index{0};
  int version{0};
  std::vector<Point> point_list;
};

struct Submap {
  int index{0};
  int version{0};
  Pose pose{};
  std::vector<Scan> scan_list;
};

#endif  // MESSAGE_QUEUE_PATTERN_TYPES_H_
