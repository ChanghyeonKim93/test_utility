#ifndef MESSAGE_HANDLER_H_
#define MESSAGE_HANDLER_H_

#include <condition_variable>
#include <deque>
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "worker_thread_pool.h"

/* Note: This is an example of observer pattern. */

using namespace std::chrono_literals;

template <class DataType>
class MessageHandler {
 public:
  explicit MessageHandler(const int max_queue_size)
      : max_queue_size_(max_queue_size), thread_pool_(4) {
    std::thread t = std::thread([this]() { RunCallbacksWhileLoop(); });
    t.detach();
  }

  void Publish(const DataType& message) {
    message_queue_.push_back(message);
    while (static_cast<int>(message_queue_.size()) > max_queue_size_)
      message_queue_.pop_front();
    cv_.notify_one();
  }

  void Subscribe(std::function<void(DataType)>&& callback_function) {
    std::lock_guard lock(mutex_);
    callback_function_list_.emplace_back(
        std::forward<std::function<void(DataType)>>(callback_function));
  }

  void RunCallbacksWhileLoop() {
    while (true) {
      std::unique_lock<std::mutex> lock(mutex_);
      const bool is_wake_up_condition = cv_.wait_for(lock, 1000ms, [&] {
        return (!callback_function_list_.empty() && !message_queue_.empty());
      });
      (void)is_wake_up_condition;

      std::optional<DataType> message = GetAndDequeueMessage();
      if (!message.has_value()) continue;

      // 각 콜백함수를 실행하고 온다.
      for (auto&& callback : callback_function_list_) {
        thread_pool_.EnqueueTask(callback, message.value());
        //     std::thread t =
        //     std::thread(callback,
        //                 message.value());
        // t.detach();
      }
    }
  }

 private:
  std::optional<DataType> GetAndDequeueMessage() {
    if (message_queue_.empty()) return std::nullopt;

    const auto item = message_queue_.front();
    message_queue_.pop_front();
    return item;
  }

 private:
  const int max_queue_size_;

  std::mutex mutex_;
  std::condition_variable cv_;

  std::deque<DataType> message_queue_;
  std::list<std::function<void(DataType)>> callback_function_list_;

  WorkerThreadPool thread_pool_;
};

#endif  // MESSAGE_HANDLER_H_