#include <vector>

#include "time_checker.h"

class Dummy {
 public:
  void Func1();
};

void Dummy::Func1() {
  CHECK_FUNCTION_TIME_FROM_THIS
  std::vector<double> a;
  for (int i = 0; i < 1000000; ++i) {
    a.push_back(i);
  }
  for (int i = 0; i < 1000000; ++i) {
    a.at(i) *= i;
  }
}

void Func2() {
  CHECK_FUNCTION_TIME_FROM_THIS
  std::vector<double> a;
  for (int i = 0; i < 10000000; ++i) a.push_back(i);
  for (int i = 0; i < 10000000; ++i) a.at(i) *= i;
}

void Func3() { CHECK_FUNCTION_TIME_FROM_THIS }

int main() {
  CHECK_FUNCTION_TIME_FROM_THIS

  Dummy d;
  d.Func1();
  Func2();
  Func2();
  Func2();
  Func2();
  for (int iter = 0; iter < 10000000; ++iter) Func3();

  TimeCheckerManager::SaveFile("fff.txt");
  return 0;
}
