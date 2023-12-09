#ifndef MESSAGE_QUEUE_PATTERN_MULTI_THREAD_EXECUTOR_H_
#define MESSAGE_QUEUE_PATTERN_MULTI_THREAD_EXECUTOR_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifndef PrintInfo
void PrintInfoImpl(const std::string& str, const std::string& func_str) {
  std::stringstream ss;
  ss << "[ThreadId:" << std::this_thread::get_id() << "][INFO] \"" << func_str
     << "()\": " << str << "\n";
  std::cerr << ss.str();
}
#define PrintInfo(str) PrintInfoImpl(str, std::string(__func__));
#endif

#ifndef ThrowError
void ThrowErrorImpl(const std::string& str, const std::string& func_str) {
  std::stringstream ss;
  ss << "[ThreadId:" << std::this_thread::get_id() << "][Error] \"" << func_str
     << "()\": " << str << "\n";
  throw std::runtime_error(ss.str());
}
#define ThrowError(str) ThrowErrorImpl(str, std::string(__func__));
#endif

class MultiThreadExecutor {
 public:
  enum class Status { kRun = 0, kKill = 1 };

 public:
  explicit MultiThreadExecutor(int num_threads) : status_(Status::kRun) {
    worker_thread_list_.reserve(num_threads);
    for (int index = 0; index < num_threads; ++index)
      worker_thread_list_.emplace_back(
          [this]() { RunProcessForWorkerThread(); });
  }

  MultiThreadExecutor(const int num_threads,
                      const std::vector<int>& processor_numbers_for_each_thread)
      : status_(Status::kRun) {
    if (num_threads !=
        static_cast<int>(processor_numbers_for_each_thread.size()))
      ThrowError("processor_numbers_for_each_thread.size() != num_threads");

    worker_thread_list_.reserve(num_threads);
    for (int index = 0; index < num_threads; ++index) {
      worker_thread_list_.emplace_back(
          [this]() { RunProcessForWorkerThread(); });
      AllocateProcessorsForEachThread(processor_numbers_for_each_thread[index],
                                      &worker_thread_list_[index]);
    }
  }

  ~MultiThreadExecutor() {
    status_ = Status::kKill;
    WakeUpAllThreads();
    for (auto& worker_thread : worker_thread_list_) {
      worker_thread.join();
      PrintInfo("Executor thread successfully joins.");
    }
  }

  template <typename Func, typename... Args>
  std::future<typename std::invoke_result<Func, Args...>::type> Execute(
      Func&& func, Args&&... args) {
    using ReturnType = typename std::invoke_result<Func, Args...>::type;
    auto task_ptr = std::make_shared<std::packaged_task<ReturnType()>>(
        std::bind(std::forward<Func>(func), std::forward<Args>(args)...));
    std::future<ReturnType> future_result = task_ptr->get_future();

    {
      std::lock_guard<std::mutex> local_lock(mutex_for_cv_);
      task_queue_.push_back([task_ptr]() { (*task_ptr)(); });
    }
    WakeUpOneThread();
    return future_result;
  }

  void PurgeAllTasks() {
    std::unique_lock<std::mutex> local_lock(mutex_for_cv_);
    task_queue_.clear();
  }

  int GetNumOfOngoingTasks() {
    std::unique_lock<std::mutex> local_lock(mutex_for_cv_);
    return num_of_ongoing_tasks_;
  }

  int GetNumOfQueuedTasks() {
    std::unique_lock<std::mutex> local_lock(mutex_for_cv_);
    return static_cast<int>(task_queue_.size());
  }

  int GetNumOfTotalThreads() {
    std::unique_lock<std::mutex> local_lock(mutex_for_cv_);
    return static_cast<int>(worker_thread_list_.size());
  }

  int GetNumOfAwaitingThreads() {
    std::unique_lock<std::mutex> local_lock(mutex_for_cv_);
    return (static_cast<int>(worker_thread_list_.size()) -
            num_of_ongoing_tasks_);
  }

 private:
  void WakeUpOneThread() { cv_.notify_one(); }
  void WakeUpAllThreads() { cv_.notify_all(); }
  bool AllocateProcessorsForEachThread(const int processor_index,
                                       std::thread* input_thread) {
    const int num_max_threads_for_this_cpu =
        static_cast<int>(std::thread::hardware_concurrency());
    if (processor_index >= num_max_threads_for_this_cpu) {
      PrintInfo("Exceed the maximum number of logical processors!");
      return false;
    }

    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(processor_index, &cpu_set);
    int result = pthread_setaffinity_np(input_thread->native_handle(),
                                        sizeof(cpu_set_t), &cpu_set);
    if (result != 0) {
      PrintInfo("Fail to allocate processor. pthread_setaffinity_np yields: " +
                std::to_string(result));
      return false;
    }
    return true;
  }
  void RunProcessForWorkerThread() {
    while (true) {
      std::unique_lock<std::mutex> local_lock(mutex_for_cv_);
      if (!cv_.wait_for(local_lock, std::chrono::microseconds(1000), [this]() {
            return (!task_queue_.empty() || (status_ == Status::kKill));
          })) {
        continue;
      }

      if (status_ == Status::kKill) break;

      std::function<void()> new_task = std::move(task_queue_.front());
      task_queue_.pop_front();
      ++num_of_ongoing_tasks_;
      local_lock.unlock();

      new_task();  // Do the job!

      local_lock.lock();
      --num_of_ongoing_tasks_;
      local_lock.unlock();
    }
    PrintInfo("While loop of executor thread is terminated.");
  }

 private:
  std::atomic<Status> status_;

  std::mutex mutex_for_cv_;
  std::condition_variable cv_;

  std::vector<std::thread> worker_thread_list_;
  std::deque<std::function<void()>> task_queue_;

  int num_of_ongoing_tasks_{0};
};

#endif  // MESSAGE_QUEUE_PATTERN_MULTI_THREAD_EXECUTOR_H_
