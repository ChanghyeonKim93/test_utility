#include "worker_thread_pool.h"

#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

using namespace std ::chrono_literals;

void PrintInfoImpl(const std::string& str, const std::string& func_str) {
  std::stringstream ss;
  ss << "[ThreadId:" << std::this_thread::get_id() << "][INFO] \"" << func_str
     << "()\": " << str << "\n";
  std::cerr << ss.str();
}
#define PrintInfo(str) PrintInfoImpl(str, std::string(__func__));

void ThrowErrorImpl(const std::string& str, const std::string& func_str) {
  std::stringstream ss;
  ss << "[ThreadId:" << std::this_thread::get_id() << "][Error] \"" << func_str
     << "()\": " << str << "\n";
  throw std::runtime_error(ss.str());
}
#define ThrowError(str) ThrowErrorImpl(str, std::string(__func__));

WorkerThreadPool::WorkerThreadPool(int num_threads_in_pool)
    : thread_pool_status_(WorkerThreadPoolStatus::kRun) {
  worker_thread_list_.reserve(num_threads_in_pool);
  for (int index = 0; index < num_threads_in_pool; ++index)
    worker_thread_list_.emplace_back([this]() { RunProcessForWorkerThread(); });
}

WorkerThreadPool::WorkerThreadPool(const int num_threads_in_pool,
                                   const std::vector<int>& cpu_affinity_numbers)
    : thread_pool_status_(WorkerThreadPoolStatus::kRun) {
  if (num_threads_in_pool != static_cast<int>(cpu_affinity_numbers.size()))
    ThrowError("cpu_affinity_numbers.size() != num_threads");

  worker_thread_list_.reserve(num_threads_in_pool);
  for (int index = 0; index < num_threads_in_pool; ++index) {
    worker_thread_list_.emplace_back([this]() { RunProcessForWorkerThread(); });
    AllocateCpuProcessorForEachThread(worker_thread_list_[index],
                                      cpu_affinity_numbers[index]);
  }
}

WorkerThreadPool::~WorkerThreadPool() {
  thread_pool_status_ = WorkerThreadPoolStatus::kKill;
  WakeUpAllThreads();
  for (auto& worker_thread : worker_thread_list_) {
    PrintInfo("--- JOINING THE THREAD");
    while (!worker_thread.joinable()) WakeUpAllThreads();
    worker_thread.join();
    PrintInfo("Joining is done!");
  }
}

void WorkerThreadPool::RunProcessForWorkerThread() {
  PrintInfo("The thread is initialized.");
  while (true) {
    std::unique_lock<std::mutex> local_lock(mutex_);
    condition_variable_.wait(local_lock, [this]() {
      return (!task_queue_.empty() ||
              (thread_pool_status_ == WorkerThreadPoolStatus::kKill));
    });

    if (thread_pool_status_ == WorkerThreadPoolStatus::kKill) {
      PrintInfo(
          "The thread captured stop sign. The thread is going to join...");
      break;
    }

    // Get a job
    std::function<void()> new_task = std::move(task_queue_.front());
    task_queue_.pop();
    ++num_of_ongoing_tasks_;
    local_lock.unlock();

    // Do the job!
    new_task();

    local_lock.lock();
    --num_of_ongoing_tasks_;
    local_lock.unlock();
  }
  PrintInfo("The thread is end.");
}

void WorkerThreadPool::EnqueueTask(std::function<void()> task) {
  if (thread_pool_status_ == WorkerThreadPoolStatus::kKill) {
    PrintInfo("Thread pool is terminated. Thread pool status is 'kKill'");
    return;
  }

  mutex_.lock();
  task_queue_.push(std::move(task));
  mutex_.unlock();

  WakeUpOneThread();
}

int WorkerThreadPool::GetNumOfOngoingTasks() {
  std::unique_lock<std::mutex> local_lock(mutex_);
  return num_of_ongoing_tasks_;
}

int WorkerThreadPool::GetNumOfQueuedTasks() {
  std::unique_lock<std::mutex> local_lock(mutex_);
  return static_cast<int>(task_queue_.size());
}

int WorkerThreadPool::GetNumOfTotalThreads() {
  std::unique_lock<std::mutex> local_lock(mutex_);
  return static_cast<int>(worker_thread_list_.size());
}

int WorkerThreadPool::GetNumOfAwaitingThreads() {
  std::unique_lock<std::mutex> local_lock(mutex_);
  return (static_cast<int>(worker_thread_list_.size()) - num_of_ongoing_tasks_);
}

bool WorkerThreadPool::AllocateCpuProcessorForEachThread(
    std::thread& input_thread, const int cpu_core_index) {
  const int num_max_threads_for_this_cpu =
      static_cast<int>(std::thread::hardware_concurrency());
  if (cpu_core_index >= num_max_threads_for_this_cpu) {
    PrintInfo("Exceed the maximum logical CPU number!");
    return false;
  }

  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  CPU_SET(cpu_core_index, &cpu_set);
  int result = pthread_setaffinity_np(input_thread.native_handle(),
                                      sizeof(cpu_set_t), &cpu_set);
  if (result != 0) {
    PrintInfo("Error calling pthread_setaffinity_np: " +
              std::to_string(result));
    return false;
  }

  PrintInfo("The thread is confined to CPU Core [" +
            std::to_string(cpu_core_index) + "]");
  return true;
}

void WorkerThreadPool::WakeUpOneThread() { condition_variable_.notify_one(); }

void WorkerThreadPool::WakeUpAllThreads() { condition_variable_.notify_all(); }