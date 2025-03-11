#include <chrono>
#include <iostream>
#include <thread>

#include "multi_thread_executor.h"

using namespace std::chrono_literals;

struct Feature {
  float x{0.0f};
  float y{0.0f};
  float response{0.0f};
  int octave{0};
};

std::vector<Feature> FindFeatures(const int num_features,
                                  const int fast_threshold,
                                  const int num_scale_level,
                                  const float scale_factor) {
  std::cerr << std::string(std::to_string(num_features) + " : START !\n");

  std::vector<Feature> feature_list(num_features);
  std::vector<unsigned long long> ss;
  unsigned long long sum = 0;
  for (unsigned long long index = 0;
       index < static_cast<unsigned long long>(num_features) * 500000;
       ++index) {
    sum += index;
    ss.push_back(sum);
  }
  for (int index = 0; index < num_features; ++index)
    feature_list[index].octave = ss[index];
  std::cerr << std::string(std::to_string(num_features) + " : Ok\n");

  return feature_list;
};

int main() {
  using FeatureList = std::vector<Feature>;

  int num_threads = 2;
  std::vector<int> select_cpu_nums(num_threads);
  select_cpu_nums[0] = 0;
  select_cpu_nums[1] = 1;

  // Async
  std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
  std::vector<std::future<FeatureList>> futures_for_async;
  futures_for_async.reserve(8);

  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 400, 20, 0, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 200, 20, 1, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 100, 20, 1, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 50, 20, 2, 0.9f));

  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 400, 20, 0, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 200, 20, 1, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 100, 20, 1, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 50, 20, 2, 0.9f));

  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 400, 20, 0, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 200, 20, 1, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 100, 20, 1, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 50, 20, 2, 0.9f));

  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 400, 20, 0, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 200, 20, 1, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 100, 20, 1, 0.9f));
  futures_for_async.emplace_back(
      std::async(std::launch::async, FindFeatures, 50, 20, 2, 0.9f));

  FeatureList all_feature_list_for_async;
  for (size_t index = 0; index < futures_for_async.size(); ++index) {
    auto feature_list = futures_for_async[index].get();
    for (size_t feature_index = 0; feature_index < feature_list.size();
         ++feature_index) {
      all_feature_list_for_async.push_back(feature_list.at(feature_index));
    }
  }
  std::cerr << "all_feature_list_for_async.size() : "
            << all_feature_list_for_async.size() << std::endl;
  std::chrono::duration<double> dt = std::chrono::system_clock::now() - t1;
  std::cout << "std::async time : " << dt.count() << "seconds" << std::endl;

  std::unique_ptr<MultiThreadExecutor> multi_thread_executor =
      std::make_unique<MultiThreadExecutor>(num_threads, select_cpu_nums);

  std::vector<std::future<FeatureList>> futures_multi_thread_executor;
  futures_multi_thread_executor.reserve(8);

  t1 = std::chrono::system_clock::now();

  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 400, 20, 0, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 200, 20, 1, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 100, 20, 1, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 50, 20, 2, 0.9f));

  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 400, 20, 0, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 200, 20, 1, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 100, 20, 1, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 50, 20, 2, 0.9f));

  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 400, 20, 0, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 200, 20, 1, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 100, 20, 1, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 50, 20, 2, 0.9f));

  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 400, 20, 0, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 200, 20, 1, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 100, 20, 1, 0.9f));
  futures_multi_thread_executor.emplace_back(
      multi_thread_executor->Execute(FindFeatures, 50, 20, 2, 0.9f));

  FeatureList all_feature_list;
  for (size_t index = 0; index < futures_multi_thread_executor.size();
       ++index) {
    auto feature_list = futures_multi_thread_executor[index].get();
    for (size_t feature_index = 0; feature_index < feature_list.size();
         ++feature_index) {
      all_feature_list.push_back(feature_list.at(feature_index));
    }
  }
  std::cerr << "all_feature_list.size() : " << all_feature_list.size()
            << std::endl;

  dt = std::chrono::system_clock::now() - t1;
  std::cout << "multi_thread_executor time : " << dt.count() << "seconds"
            << std::endl;

  t1 = std::chrono::system_clock::now();
  FindFeatures(400, 20, 0, 0.9f);
  FindFeatures(200, 20, 1, 0.9f);
  FindFeatures(100, 20, 2, 0.9f);
  FindFeatures(50, 20, 2, 0.9f);

  FindFeatures(400, 20, 0, 0.9f);
  FindFeatures(200, 20, 1, 0.9f);
  FindFeatures(100, 20, 2, 0.9f);
  FindFeatures(50, 20, 2, 0.9f);
  dt = std::chrono::system_clock::now() - t1;
  std::cout << "naive sequential execution time : " << dt.count() << "seconds"
            << std::endl;

  std::cerr << "std::thread::hardware_concurrency(): "
            << std::thread::hardware_concurrency() << std::endl;

  return 0;
}