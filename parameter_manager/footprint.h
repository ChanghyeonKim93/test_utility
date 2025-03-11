#ifndef FOOTPRINT_H_
#define FOOTPRINT_H_

#include <vector>

class Footprint {
 public:
  enum class Type { kUnknown, kRectangular, kCircular, kPolygonal };
  Type type{Type::kUnknown};
  double height;
  struct {
    double x;
    double y;
  } offset;
};

class RectangularFootprint : public Footprint {
 public:
  Type type{Type::kRectangular};
  double length{0.0};
  double width{0.0};
};

class CircularFootprint : public Footprint {
 public:
  Type type{Type::kCircular};
  double radius{0.0};
};

class PolygonalFootprint : public Footprint {
 public:
  Type type{Type::kPolygonal};
  std::vector<double> x_list;
  std::vector<double> y_list;
};

#endif  // FOOTPRINT_BASE_H_