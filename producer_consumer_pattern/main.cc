#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>

#include "consumer.h"

int main() {
  std::atomic<ThreadStatus> thread_status{ThreadStatus::kRunning};
  std::atomic<TaskStatus> task_status{TaskStatus::kIdle};
  std::condition_variable cv;

  Consumer consumer_1(&thread_status, &task_status, &cv);

  task_status = TaskStatus::kDoTask1;
  cv.notify_all();
  std::this_thread::sleep_for(500ms);

  task_status = TaskStatus::kDoTask2;
  cv.notify_all();
  std::this_thread::sleep_for(500ms);

  task_status = TaskStatus::kDoTask3;
  cv.notify_one();
  std::this_thread::sleep_for(500ms);

  thread_status = ThreadStatus::kKill;
  cv.notify_one();

  std::cerr << "Producer node is terminated.\n";

  return 0;
}