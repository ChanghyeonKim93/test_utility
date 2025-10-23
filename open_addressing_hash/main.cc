#include <chrono>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <random>
#include <unordered_map>
#include <vector>

// #include "open_addressing_hash_map.h"
#include "soa_oa_hash_map.h"

struct Vec3i {
  int x{0};
  int y{0};
  int z{0};
  Vec3i() = default;
  Vec3i(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}
  bool operator==(const Vec3i& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

template <>
struct std::hash<Vec3i> {
  size_t operator()(const Vec3i& v) const {
    // FNV-1a 기반 해시 구현 - 분산 향상
    constexpr size_t FNV_PRIME = 16777619;
    constexpr size_t OFFSET_BASIS = 2166136261;

    size_t hash = OFFSET_BASIS;
    hash ^= static_cast<size_t>(v.x);
    hash *= FNV_PRIME;
    hash ^= static_cast<size_t>(v.y);
    hash *= FNV_PRIME;
    hash ^= static_cast<size_t>(v.z);
    hash *= FNV_PRIME;

    return hash;
  }
};

struct Vec3iHash {
  size_t operator()(const Vec3i& v) const {
    // FNV-1a 기반 해시 구현 - 분산 향상
    constexpr size_t FNV_PRIME = 16777619;
    constexpr size_t OFFSET_BASIS = 2166136261;

    size_t hash = OFFSET_BASIS;
    hash ^= static_cast<size_t>(v.x);
    hash *= FNV_PRIME;
    hash ^= static_cast<size_t>(v.y);
    hash *= FNV_PRIME;
    hash ^= static_cast<size_t>(v.z);
    hash *= FNV_PRIME;

    return hash;
  }
};

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

struct Mat3x3 {
  double data[3][3]{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  Mat3x3() = default;
};

struct NormalDistribution {
  int id{0};
  Vec3 mean;
  Mat3x3 covariance;
  Vec3 plane_normal_vector;
  bool is_singular{false};
  bool is_planar{false};
};

// 시간 측정용 헬퍼 클래스
class Timer {
 public:
  Timer() : start_time(std::chrono::high_resolution_clock::now()) {}

  double elapsedMilliseconds() {
    auto end_time = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end_time - start_time)
        .count();
  }

 private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
};

// 랜덤 Vec3i 생성
Vec3i generateRandomVec3i(std::mt19937& gen) {
  std::uniform_int_distribution<int> dist(0, 2000);
  return Vec3i(dist(gen), dist(gen), dist(gen));
}

// 랜덤 NormalDistribution 생성
NormalDistribution generateRandomNDT(int id) {
  NormalDistribution ndt;
  ndt.id = id;
  return ndt;
}

int main() {
  const size_t NUM_ELEMENTS = 1000000;
  const size_t NUM_LOOKUPS = 50000;
  const size_t INITIAL_CAPACITY = NUM_ELEMENTS / 0.5;  // 50% 로드 팩터로 시작

  // 난수 생성기 초기화
  std::random_device rd;
  std::mt19937 gen(rd());

  std::cout << "===== 해시맵 성능 비교 =====" << std::endl;
  std::cout << "요소 개수: " << NUM_ELEMENTS << std::endl;
  std::cout << "조회 횟수: " << NUM_LOOKUPS << std::endl << std::endl;

  // 테스트에 사용할 데이터 생성
  std::cout << "테스트 데이터 생성 중..." << std::endl;
  std::vector<std::pair<Vec3i, NormalDistribution>> data;
  data.reserve(NUM_ELEMENTS);

  for (size_t i = 0; i < NUM_ELEMENTS; ++i) {
    Vec3i key = generateRandomVec3i(gen);
    NormalDistribution ndt = generateRandomNDT(static_cast<int>(i));
    data.emplace_back(key, ndt);
  }

  // 조회할 키 목록 생성 (기존 데이터에서 랜덤하게 선택)
  std::vector<Vec3i> lookup_keys;
  lookup_keys.reserve(NUM_LOOKUPS);
  std::uniform_int_distribution<size_t> index_dist(0, NUM_ELEMENTS - 1);

  for (size_t i = 0; i < NUM_LOOKUPS; ++i) {
    size_t index = index_dist(gen);
    lookup_keys.push_back(data[index].first);
  }

  std::cout << "데이터 생성 완료" << std::endl << std::endl;

  // 1. std::unordered_map 테스트
  {
    std::unordered_map<Vec3i, NormalDistribution> std_map;
    std_map.reserve(INITIAL_CAPACITY);

    // 삽입 시간 측정
    Timer insert_timer;
    for (const auto& [key, value] : data) {
      std_map.insert({key, value});
    }
    double std_insert_time = insert_timer.elapsedMilliseconds();

    // 조회 시간 측정
    Timer lookup_timer;
    size_t found_count = 0;
    for (const auto& key : lookup_keys) {
      auto it = std_map.find(key);
      if (it != std_map.end()) {
        found_count++;
        // 컴파일러 최적화 방지를 위한 더미 작업
        volatile int dummy = it->second.id;
      }
    }
    double std_lookup_time = lookup_timer.elapsedMilliseconds();

    std::cout << "std::unordered_map 결과:" << std::endl;
    std::cout << "삽입 시간: " << std::fixed << std::setprecision(2)
              << std_insert_time << " ms" << std::endl;
    std::cout << "조회 시간: " << std::fixed << std::setprecision(2)
              << std_lookup_time << " ms" << std::endl;
    std::cout << "찾은 요소 수: " << found_count << "/" << NUM_LOOKUPS
              << std::endl;
    std::cout << "맵 크기: " << std_map.size() << std::endl;
    std::cout << "버킷 수: " << std_map.bucket_count() << std::endl;
    std::cout << "로드 팩터: " << std_map.load_factor() << std::endl
              << std::endl;
  }

  // 2. OpenAddressingHash 테스트
  {
    OpenAddressingHash<Vec3i, NormalDistribution, std::hash<Vec3i>> custom_map(
        INITIAL_CAPACITY);
    // OpenAddressingHash<Vec3i, NormalDistribution>
    // custom_map(INITIAL_CAPACITY);

    // 삽입 시간 측정
    Timer insert_timer;
    for (const auto& [key, value] : data) {
      custom_map.insert(key, value);
    }
    double custom_insert_time = insert_timer.elapsedMilliseconds();

    // 조회 시간 측정
    Timer lookup_timer;
    size_t found_count = 0;
    for (const auto& key : lookup_keys) {
      auto it = custom_map.find(key);
      if (it != custom_map.end()) {
        found_count++;
        // 컴파일러 최적화 방지를 위한 더미 작업
        volatile int dummy = (*it).second.id;
      }
    }
    double custom_lookup_time = lookup_timer.elapsedMilliseconds();

    std::cout << "OpenAddressingHash 결과:" << std::endl;
    std::cout << "삽입 시간: " << std::fixed << std::setprecision(2)
              << custom_insert_time << " ms" << std::endl;
    std::cout << "조회 시간: " << std::fixed << std::setprecision(2)
              << custom_lookup_time << " ms" << std::endl;
    std::cout << "찾은 요소 수: " << found_count << "/" << NUM_LOOKUPS
              << std::endl;

    // 모든 요소 개수 세기
    size_t count = 0;
    for (const auto& [key, value] : custom_map) {
      count++;
    }
    std::cout << "맵 크기: " << count << std::endl;
    std::cout << "로드 팩터: " << custom_map.get_load_factor() << std::endl
              << std::endl;
  }

  // 3. 성능 비교 요약
  std::cout << "===== 성능 비교 요약 =====" << std::endl;
  std::cout << "각 테스트는 " << NUM_ELEMENTS << "개의 요소를 삽입하고 "
            << NUM_LOOKUPS << "개의 요소를 조회했습니다." << std::endl;
  std::cout << "자세한 시간은 위의 결과를 참조하세요." << std::endl;

  return 0;
}