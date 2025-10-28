#ifndef TYPES_H_
#define TYPES_H_

#include <cstdint>

struct Vec3i {
  int x{0};
  int y{0};
  int z{0};
  Vec3i() = default;
  Vec3i(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}
  bool operator==(const Vec3i& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator!=(const Vec3i& other) const { return !(*this == other); }

  inline uint64_t GetMortonCode() const {
    auto interleave_bits = [](const uint32_t x) -> uint64_t {
      uint64_t result = static_cast<uint64_t>(x);
      result = (result | (result << 16)) & 0x00FF0000FF0000FF;
      result = (result | (result << 8)) & 0xF00F00F00F00F00F;
      result = (result | (result << 4)) & 0x3063063063063063;
      result = (result | (result << 2)) & 0x1249249249249249;
      return result;
    };
    return (interleave_bits(x)) | (interleave_bits(y) << 1) |
           (interleave_bits(z) << 2);
  }
};

#endif  // TYPES_H_
