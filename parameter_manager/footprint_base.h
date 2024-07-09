#ifndef FOOTPRINT_BASE_H_
#define FOOTPRINT_BASE_H_

class FootprintBase {
  enum class Type { kRectangular, kCircular, kPolygonal };

 public:
 private:
};

class RectangularFootprint : public FootprintBase {
 public:
  double GetLength() const { return length; }
  double GetWidth() const { return width; }
  double GetHeight() const { return height; }
  double GetOffsetX() const { return offset_x; }
  double GetOffsetY() const { return offset_y; }
  double GetOffsetZ() const { return offset_z; }

 private:
  double length;
  double width;
  double height;
  double offset_x;
  double offset_y;
  double offset_z;

  double radius;
};

#endif  // FOOTPRINT_BASE_H_