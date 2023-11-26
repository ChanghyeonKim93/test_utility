#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <sstream>
#include <thread>

enum class ThreadStatus { kRunning = 0, kKill = 1 };
enum class TaskStatus { kIdle = 0, kDoTask1, kDoTask2, kDoTask3 };

using namespace std::chrono_literals;

class Consumer {
 public:
  Consumer(std::atomic<ThreadStatus>* thread_status,
           std::atomic<TaskStatus>* task_status,
           std::condition_variable* shared_cv)
      : thread_status_(thread_status),
        task_status_(task_status),
        shared_cv_(shared_cv),
        thread_([&]() { RunThread(); }) {}

  ~Consumer() {
    *thread_status_ = ThreadStatus::kKill;
    *task_status_ = TaskStatus::kIdle;
    if (thread_.joinable()) thread_.join();
    std::cerr << "Consumer thread is terminated.\n";
  }

 private:
  void DoTask1() { std::cerr << "Do Task1\n"; }
  void DoTask2() { std::cerr << "Do Task2\n"; }
  void DoTask3() {
    std::cerr << "Do Task3\n";
    std::this_thread::sleep_for(1000ms);
  }

  void RunThread() {
    while (true) {
      std::unique_lock<std::mutex> lock(mutex_);

      shared_cv_->wait_for(lock, 2000ms, [&] {
        // shared_cv_->wait(lock, [&] {
        bool be_awaken = false;
        be_awaken |= *thread_status_ == ThreadStatus::kKill;
        be_awaken |= *task_status_ == TaskStatus::kDoTask1;
        be_awaken |= *task_status_ == TaskStatus::kDoTask2;
        be_awaken |= *task_status_ == TaskStatus::kDoTask3;
        return be_awaken;
      });

      if (*thread_status_ == ThreadStatus::kKill) {
        std::cerr << "Kill the consumer thread.\n";
        break;
      }

      // Do the job
      switch (*task_status_) {
        case TaskStatus::kDoTask1: {
          DoTask1();
          break;
        }
        case TaskStatus::kDoTask2: {
          DoTask2();
          break;
        }
        case TaskStatus::kDoTask3: {
          DoTask3();
          break;
        }
        default: {
          break;
        }
      }

      *task_status_ = TaskStatus::kIdle;
    }
  }

 private:
  std::atomic<ThreadStatus>* thread_status_;
  std::atomic<TaskStatus>* task_status_;
  std::condition_variable* shared_cv_;
  std::thread thread_;
  std::mutex mutex_;
};
