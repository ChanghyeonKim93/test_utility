#include <iostream>

struct Vec3i {
  int x{0};
  int y{0};
  int z{0};
};

uint64_t InterleaveBits(const uint32_t x) {
  uint64_t result = static_cast<uint64_t>(x);
  result = (result | (result << 16)) & 0x030000FF;
  result = (result | (result << 8)) & 0x0300F00F;
  result = (result | (result << 4)) & 0x030C30C3;
  result = (result | (result << 2)) & 0x09249249;
  return result;
}

uint64_t GetMortonCode(const Vec3i& point) {
  return (InterleaveBits(point.x) << 2) | (InterleaveBits(point.y) << 1) |
         InterleaveBits(point.z);
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

  return 0;
}