#include "optimizer/time_checker.h"

namespace optimizer {

TimeChecker::TimeChecker(const std::string& timer_name)
    : timer_name_(timer_name) {}

TimeChecker::~TimeChecker() {}

double TimeChecker::Start(const bool flag_verbose) {
  start_ = std::chrono::high_resolution_clock::now();
  intermediate_ = start_;
  std::chrono::high_resolution_clock::duration gap = intermediate_ - start_;
  double gap_in_sec = (double)(gap / std::chrono::microseconds(1)) * 0.000001;
  if (flag_verbose)
    std::cout << "[" << timer_name_ << "]    start at: " << gap_in_sec
              << " [sec]\n";

  return gap_in_sec;
}

double TimeChecker::GetTimeFromStart(const bool flag_verbose) {
  intermediate_ = std::chrono::high_resolution_clock::now();

  std::chrono::high_resolution_clock::duration gap = intermediate_ - start_;
  double gap_in_sec = (double)(gap / std::chrono::microseconds(1)) * 0.000001;
  if (flag_verbose)
    std::cout << "[" << timer_name_ << "] lap time at: " << gap_in_sec
              << " [sec]\n";
  return gap_in_sec;
}

double TimeChecker::GetTimeFromLatest(const bool flag_verbose) {
  std::chrono::high_resolution_clock::time_point time_now =
      std::chrono::high_resolution_clock::now();

  std::chrono::high_resolution_clock::duration gap = time_now - intermediate_;
  double gap_in_sec = (double)(gap / std::chrono::microseconds(1)) * 0.000001;
  if (flag_verbose)
    std::cout << "[" << timer_name_ << "] lap time at: " << gap_in_sec
              << " [sec]\n";

  intermediate_ = time_now;

  return gap_in_sec;
}

double TimeChecker::Stop(const bool flag_verbose) {
  end_ = std::chrono::high_resolution_clock::now();

  std::chrono::high_resolution_clock::duration gap = end_ - start_;
  double gap_in_sec = (double)(gap / std::chrono::microseconds(1)) * 0.000001;
  if (flag_verbose)
    std::cout << "[" << timer_name_ << "]      end at: " << gap_in_sec
              << " [ms]\n";

  return gap_in_sec;
}

void TimeChecker::tic() {
  tic_start_time_ = std::chrono::high_resolution_clock::now();
}

double TimeChecker::toc(bool flag_verbose) {
  auto finish = std::chrono::high_resolution_clock::now();
  auto gap = finish - tic_start_time_;
  const double elapsed_time =
      static_cast<double>((gap / std::chrono::microseconds(1)) / 1000000.0);
  if (flag_verbose)
    std::cout << "Execution time: " << elapsed_time << "[sec]\n";
  return elapsed_time;
}

// Get current data/time, format is yyyy-mm-dd.hh:mm:ss
const std::string TimeChecker::GetCurrentDateAndTime() {
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about data/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d_%H_%M_%S", &tstruct);

  return buf;
}

}  // namespace optimizer