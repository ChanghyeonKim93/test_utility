#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>
#include "time_checker.h"

// Q16.16 고정 소수점 연산 클래스
class Q16_16 {
 private:
  int32_t value;  // 16비트 정수부 + 16비트 소수부
  static constexpr int FRACTION_BITS = 16;
  static constexpr int32_t ONE = 1 << FRACTION_BITS;  // 1.0 값 (65536)

 public:
  Q16_16() : value(0) {}

  // float에서 Q16.16으로 변환
  Q16_16(float f) : value(static_cast<int32_t>(f * ONE)) {}

  // Q16.16에서 float으로 변환
  float to_float() const { return static_cast<float>(value) / ONE; }

  // 덧셈
  Q16_16 operator+(const Q16_16& other) const {
    Q16_16 result;
    result.value = value + other.value;
    return result;
  }

  // 곱셈
  Q16_16 operator*(const Q16_16& other) const {
    Q16_16 result;
    // Q16.16 곱셈시 32비트 범위를 벗어날 수 있으므로 64비트로 확장
    int64_t mul = static_cast<int64_t>(value) * other.value;
    result.value = static_cast<int32_t>(mul >> FRACTION_BITS);
    return result;
  }
};

// Q8.8 고정 소수점 연산 클래스
class Q8_8 {
 private:
  int16_t value;  // 8비트 정수부 + 8비트 소수부
  static constexpr int FRACTION_BITS = 8;
  static constexpr int16_t ONE = 1 << FRACTION_BITS;  // 1.0 값 (256)

 public:
  Q8_8() : value(0) {}

  // float에서 Q8.8로 변환
  Q8_8(float f) : value(static_cast<int16_t>(f * ONE)) {}

  // Q8.8에서 float으로 변환
  float to_float() const { return static_cast<float>(value) / ONE; }

  // 덧셈
  Q8_8 operator+(const Q8_8& other) const {
    Q8_8 result;
    result.value = value + other.value;
    return result;
  }

  // 곱셈
  Q8_8 operator*(const Q8_8& other) const {
    Q8_8 result;
    // Q8.8 곱셈시 16비트 범위를 벗어날 수 있으므로 32비트로 확장
    int32_t mul = static_cast<int32_t>(value) * other.value;
    result.value = static_cast<int16_t>(mul >> FRACTION_BITS);
    return result;
  }
};

int main() {
  // 랜덤 숫자 생성기 초기화
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist(-5.0f, 5.0f);

  // 결과를 저장할 벡터
  std::vector<float> float_results;
  std::vector<float> q16_16_results;
  std::vector<float> q8_8_results;

  // 테스트 반복 횟수
  const int NUM_TESTS = 100;

  std::cout << std::setprecision(6) << std::fixed;
  std::cout << "=== (((a+b)*c+d)*e) 계산 결과 비교 ===" << std::endl;
  std::cout << "     a      b      c      d      e      float     Q16.16     "
               "Q8.8      오차(Q16.16)  오차(Q8.8)"
            << std::endl;

  // 여러 케이스 테스트
  for (int i = 0; i < NUM_TESTS; ++i) {
    // -120~120 사이의 랜덤 값 생성
    float a = dist(gen);
    float b = dist(gen);
    float c = dist(gen);
    float d = dist(gen);
    float e = dist(gen);

    // 1. float 계산

    float float_result = (((a + b) * c + d) * e);

    // 2. Q16.16 계산
    Q16_16 qa(a), qb(b), qc(c), qd(d), qe(e);
    float q16_16_result = (((qa + qb) * qc + qd) * qe).to_float();

    // 3. Q8.8 계산
    Q8_8 sa(a), sb(b), sc(c), sd(d), se(e);
    float q8_8_result = (((sa + sb) * sc + sd) * se).to_float();

    // 결과 저장
    float_results.push_back(float_result);
    q16_16_results.push_back(q16_16_result);
    q8_8_results.push_back(q8_8_result);

    // 오차 계산
    float error_q16_16 = std::abs(q16_16_result - float_result);
    float error_q8_8 = std::abs(q8_8_result - float_result);
    float rel_error_q16_16 =
        float_result != 0 ? error_q16_16 / std::abs(float_result) * 100.0f
                          : error_q16_16;
    float rel_error_q8_8 = float_result != 0
                               ? error_q8_8 / std::abs(float_result) * 100.0f
                               : error_q8_8;

    // 결과 출력
    std::cout << std::setw(7) << a << " " << std::setw(7) << b << " "
              << std::setw(7) << c << " " << std::setw(7) << d << " "
              << std::setw(7) << e << " " << std::setw(10) << float_result
              << " " << std::setw(10) << q16_16_result << " " << std::setw(10)
              << q8_8_result << " " << std::setw(7) << rel_error_q16_16 << "%"
              << " " << std::setw(7) << rel_error_q8_8 << "%" << std::endl;
  }

  // 통계 정보 계산
  float max_error_q16_16 = 0.0f, max_error_q8_8 = 0.0f;
  float avg_error_q16_16 = 0.0f, avg_error_q8_8 = 0.0f;

  for (int i = 0; i < NUM_TESTS; ++i) {
    float error_q16_16 = std::abs(q16_16_results[i] - float_results[i]);
    float error_q8_8 = std::abs(q8_8_results[i] - float_results[i]);

    max_error_q16_16 = std::max(max_error_q16_16, error_q16_16);
    max_error_q8_8 = std::max(max_error_q8_8, error_q8_8);

    avg_error_q16_16 += error_q16_16;
    avg_error_q8_8 += error_q8_8;
  }

  avg_error_q16_16 /= NUM_TESTS;
  avg_error_q8_8 /= NUM_TESTS;

  std::cout << "\n=== 오차 통계 ===" << std::endl;
  std::cout << "Q16.16 최대 오차: " << max_error_q16_16 << std::endl;
  std::cout << "Q16.16 평균 오차: " << avg_error_q16_16 << std::endl;
  std::cout << "Q8.8 최대 오차: " << max_error_q8_8 << std::endl;
  std::cout << "Q8.8 평균 오차: " << avg_error_q8_8 << std::endl;

  return 0;
}