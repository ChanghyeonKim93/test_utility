#ifndef WORKER_THREAD_POOL_H_
#define WORKER_THREAD_POOL_H_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

class WorkerThreadPool {
 public:
  enum class WorkerThreadPoolStatus { kRun = 0, kKill = 1 };

 public:
  explicit WorkerThreadPool(const int num_thread);
  explicit WorkerThreadPool(const int num_thread,
                            const std::vector<int>& cpu_affinity_numbers);
  ~WorkerThreadPool();
  template <typename Func, typename... Args>
  void EnqueueTask(Func&& func, Args&&... args);
  template <typename Func, typename... Args>
  std::future<typename std::invoke_result<Func, Args...>::type>
  EnqueueTaskAndGetResultInFuture(Func&& func, Args&&... args);
  int GetNumOfOngoingTasks();
  int GetNumOfQueuedTasks();
  int GetNumOfTotalThreads();
  int GetNumOfAwaitingThreads();

 private:
  void WakeUpOneThread();
  void WakeUpAllThreads();
  bool AllocateCpuProcessorForEachThread(std::thread& input_thread,
                                         const int cpu_core_index);
  void RunProcessForWorkerThread();

 private:
  std::vector<std::thread> worker_thread_list_;
  std::queue<std::function<void()>> task_queue_;

  std::mutex mutex_;
  std::condition_variable condition_variable_;
  std::atomic<WorkerThreadPoolStatus> thread_pool_status_;

  int num_of_ongoing_tasks_{0};
};

template <typename Func, typename... Args>
void WorkerThreadPool::EnqueueTask(Func&& func, Args&&... args) {
  using ReturnType = typename std::invoke_result<Func, Args...>::type;
  auto task_ptr = std::make_shared<std::packaged_task<ReturnType()>>(
      std::bind(std::forward<Func>(func), std::forward<Args>(args)...));

  mutex_.lock();
  task_queue_.push([task_ptr]() { (*task_ptr)(); });
  mutex_.unlock();

  WakeUpOneThread();
}

template <typename Func, typename... Args>
std::future<typename std::invoke_result<Func, Args...>::type>  // from C++17
WorkerThreadPool::EnqueueTaskAndGetResultInFuture(Func&& func, Args&&... args) {
  using ReturnType = typename std::invoke_result<Func, Args...>::type;

  auto task_ptr = std::make_shared<std::packaged_task<ReturnType()>>(
      std::bind(std::forward<Func>(func), std::forward<Args>(args)...));
  std::future<ReturnType> future_result = task_ptr->get_future();
  // lhs: std::future, rhs: std::promise

  mutex_.lock();
  task_queue_.push([task_ptr]() { (*task_ptr)(); });
  mutex_.unlock();

  WakeUpOneThread();

  return future_result;
}

#endif