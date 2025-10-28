#include <iostream>

struct Vec3i {
  int x{0};
  int y{0};
  int z{0};
};

uint64_t GetMortonCode(const Vec3i& point) {
  auto interleave_bits = [](const uint32_t x) -> uint64_t {
    uint64_t result = static_cast<uint64_t>(x);
    result = (result | (result << 16)) & 0x00FF0000FF0000FF;
    result = (result | (result << 8)) & 0xF00F00F00F00F00F;
    result = (result | (result << 4)) & 0x3063063063063063;
    result = (result | (result << 2)) & 0x1249249249249249;
    return result;
  };
  return (interleave_bits(point.x)) | (interleave_bits(point.y) << 1) |
         (interleave_bits(point.z) << 2);
}

Vec3i DecodeMortonCode(uint64_t morton_code) {
  auto decode = [](uint64_t code) {
    uint64_t x = code & 0x1249249249249249;
    x = (x | (x >> 2)) & 0x3063063063063063;
    x = (x | (x >> 4)) & 0xF00F00F00F00F00F;
    x = (x | (x >> 8)) & 0x00FF0000FF0000FF;
    x = (x | (x >> 16)) & 0x00000000FFFFFFFF;
    return static_cast<uint32_t>(x);
  };
  Vec3i point;
  point.x = decode(morton_code);
  point.y = decode(morton_code >> 1);
  point.z = decode(morton_code >> 2);
  return point;
}

int main(int, char**) {
  std::cout << "Morton Code Example" << std::endl;
  Vec3i p1{1, 0, 0};
  Vec3i p2{0, 1, 0};
  Vec3i p3{0, 0, 1};

  std::cout << "Point (1,0,0) Morton Code: " << GetMortonCode(p1) << std::endl;
  std::cout << "Point (0,1,0) Morton Code: " << GetMortonCode(p2) << std::endl;
  std::cout << "Point (0,0,1) Morton Code: " << GetMortonCode(p3) << std::endl;

  Vec3i p_end{5000, 5000, 5000};
  std::cout << "Point (5000,5000,5000) Morton Code: " << GetMortonCode(p_end)
            << std::endl;

  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      for (int z = 0; z < 4; ++z) {
        Vec3i p{x, y, z};
        std::cout << "Point (" << x << "," << y << "," << z
                  << ") Morton Code: " << GetMortonCode(p) << std::endl;
        std::cout << "Decoded back: ";
        Vec3i decoded = DecodeMortonCode(GetMortonCode(p));
        std::cout << "(" << decoded.x << "," << decoded.y << "," << decoded.z
                  << ")" << std::endl;
      }
    }
  }

  return 0;
}