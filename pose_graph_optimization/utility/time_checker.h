#ifndef UTILITY_TIME_CHECKER_H_
#define UTILITY_TIME_CHECKER_H_

#include <chrono>
#include <iostream>
#include <string>

namespace utility {

class TimeChecker {
 public:
  TimeChecker(const std::string& timer_checker_name);
  ~TimeChecker();

 public:
  double Start(const bool flag_verbose = false);
  double GetTimeFromStart(const bool flag_verbose = false);
  double GetTimeFromLatest(const bool flag_verbose = false);
  double Stop(const bool flag_verbose = false);

 private:
  std::string timer_name_;

  std::chrono::high_resolution_clock::time_point start_;
  std::chrono::high_resolution_clock::time_point intermediate_;
  std::chrono::high_resolution_clock::time_point end_;

 public:
  inline static std::chrono::high_resolution_clock::time_point tic_start_time_ =
      std::chrono::high_resolution_clock::now();

  static void tic();
  static double toc(bool flag_verbose);  // elapsed time from "tic()" in ms.
  static const std::string GetCurrentDateAndTime();  // yyyy-mm-dd.hh:mm:ss
};

}  // namespace utility

#endif  // UTILITY_TIME_CHECKER_H_