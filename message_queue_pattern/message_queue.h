#ifndef MESSAGE_QUEUE_H_
#define MESSAGE_QUEUE_H_

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

/* Note: This is an example of observer pattern. */

using namespace std::chrono_literals;

template <class DataType>
class MessageQueue {
 public:
  explicit MessageQueue(const int max_queue_size)
      : max_queue_size_(max_queue_size) {
    std::thread t = std::thread([this]() { RunCallbacksWhileLoop(); });
    t.detach();
  }

  void Publish(const DataType& message) {
    message_queue_.push_back(message);
    while (static_cast<int>(message_queue_.size()) > max_queue_size_)
      message_queue_.pop_front();
    condvar_.notify_one();
  }

  void Subscribe(std::function<void(DataType)>&& callback_function) {
    std::lock_guard lock(mutex_);
    callback_function_list_.emplace_back(
        std::forward<std::function<void(DataType)>>(callback_function));
  }

  // void Unsubscribe(std::function<void(DataType)>&& callback_function) {
  //   std::lock_guard lock(mutex_);
  //   callback_function_list_.erase(
  //       std::forward<std::function<void(DataType)>>(callback_function));
  // }

  void RunCallbacksWhileLoop() {
    while (true) {
      std::unique_lock<std::mutex> lock(mutex_);
      const bool is_wake_up_condition = condvar_.wait_for(lock, 1000ms, [&] {
        return (!callback_function_list_.empty() && !message_queue_.empty());
      });
      (void)is_wake_up_condition;
      // if (!is_wake_up_condition) std::cerr << "Message queue timed out. \n";

      std::optional<DataType> message = GetAndDequeueMessage();
      if (!message.has_value()) continue;

      // 각 콜백함수를 실행하고 온다.
      for (auto&& callback : callback_function_list_) {
        std::thread t = std::thread(
            callback, message.value());  // thread pool 태워서 보내야 할듯...
        t.detach();
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
  std::string topic_name_;

  std::mutex mutex_;
  std::condition_variable condvar_;

  std::deque<DataType> message_queue_;
  std::list<std::function<void(DataType)>> callback_function_list_;
};

#endif  // MESSAGE_QUEUE_H_