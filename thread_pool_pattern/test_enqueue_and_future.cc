#include <chrono>
#include <iostream>

#include "worker_thread_pool.h"

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
  std::vector<Feature> feature_list;
  feature_list.resize(num_features);
  std::this_thread::sleep_for(std::chrono::milliseconds(num_features));

  return feature_list;
};

int main() {
  int num_threads = 4;
  std::vector<int> select_cpu_nums(num_threads);
  select_cpu_nums[0] = 0;
  select_cpu_nums[1] = 1;
  select_cpu_nums[2] = 2;
  select_cpu_nums[3] = 3;

  // ThreadPool pool(num_threads, select_cpu_nums);
  std::unique_ptr<WorkerThreadPool> worker_thread_pool =
      std::make_unique<WorkerThreadPool>(num_threads);

  using FeatureList = std::vector<Feature>;
  std::vector<std::future<FeatureList>> vector_of_future_feature_list;
  vector_of_future_feature_list.reserve(8);

  std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
  vector_of_future_feature_list.emplace_back(
      worker_thread_pool->EnqueueTaskAndGetResultInFuture(FindFeatures, 300, 20,
                                                          0, 0.9f));
  vector_of_future_feature_list.emplace_back(
      worker_thread_pool->EnqueueTaskAndGetResultInFuture(FindFeatures, 200, 20,
                                                          1, 0.9f));
  vector_of_future_feature_list.emplace_back(
      worker_thread_pool->EnqueueTaskAndGetResultInFuture(FindFeatures, 100, 20,
                                                          1, 0.9f));
  vector_of_future_feature_list.emplace_back(
      worker_thread_pool->EnqueueTaskAndGetResultInFuture(FindFeatures, 50, 20,
                                                          2, 0.9f));
  vector_of_future_feature_list.emplace_back(
      worker_thread_pool->EnqueueTaskAndGetResultInFuture(FindFeatures, 300, 20,
                                                          0, 0.9f));
  vector_of_future_feature_list.emplace_back(
      worker_thread_pool->EnqueueTaskAndGetResultInFuture(FindFeatures, 200, 20,
                                                          1, 0.9f));
  vector_of_future_feature_list.emplace_back(
      worker_thread_pool->EnqueueTaskAndGetResultInFuture(FindFeatures, 100, 20,
                                                          1, 0.9f));
  vector_of_future_feature_list.emplace_back(
      worker_thread_pool->EnqueueTaskAndGetResultInFuture(FindFeatures, 50, 20,
                                                          2, 0.9f));

  FeatureList all_feature_list;
  for (size_t index = 0; index < 8; ++index) {
    auto feature_list = vector_of_future_feature_list[index].get();
    for (size_t feature_index = 0; feature_index < feature_list.size();
         ++feature_index) {
      all_feature_list.push_back(feature_list.at(feature_index));
    }
  }
  std::cerr << "all_feature_list.size() : " << all_feature_list.size()
            << std::endl;

  std::chrono::duration<double> dt = std::chrono::system_clock::now() - t1;
  std::cout << "for문을 돌리는데 걸리는 시간(초) : " << dt.count() << "seconds"
            << std::endl;

  t1 = std::chrono::system_clock::now();
  FindFeatures(300, 20, 0, 0.9f);
  FindFeatures(200, 20, 1, 0.9f);
  FindFeatures(100, 20, 2, 0.9f);
  FindFeatures(50, 20, 2, 0.9f);
  FindFeatures(300, 20, 0, 0.9f);
  FindFeatures(200, 20, 1, 0.9f);
  FindFeatures(100, 20, 2, 0.9f);
  FindFeatures(50, 20, 2, 0.9f);
  dt = std::chrono::system_clock::now() - t1;
  std::cout << "for문을 돌리는데 걸리는 시간(초) : " << dt.count() << "seconds"
            << std::endl;

  return 0;
}