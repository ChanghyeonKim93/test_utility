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

#include "multi_thread_executor.h"

/* Note: This is an example of observer pattern. */

using namespace std::chrono_literals;

template <class DataType>
class MessageHandler {
 public:
  enum class Status { kRun = 0, kKill };

 public:
  explicit MessageHandler(const int max_queue_size)
      : max_queue_size_(max_queue_size),
        multi_thread_executor_(std::make_unique<MultiThreadExecutor>(2)) {
    thread_for_callback_manager_ =
        std::thread([this]() { RunCallbackManagerWhileLoop(); });
  }

  ~MessageHandler() {
    status_ = Status::kKill;
    callback_function_list_.clear();
    message_queue_.clear();
    cv_.notify_all();
    thread_for_callback_manager_.join();
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

  void RunCallbackManagerWhileLoop() {
    while (true) {
      std::unique_lock<std::mutex> lock(mutex_);
      if (!cv_.wait_for(lock, 1000ms, [&] {
            return (
                (status_ == Status::kKill) ||
                (!callback_function_list_.empty() && !message_queue_.empty()));
          })) {
        continue;
      }
      if (status_ == Status::kKill) break;

      std::optional<DataType> message = GetAndDequeueMessage();
      if (!message.has_value()) continue;  // no message

      // 각 콜백함수를 실행하고 온다.
      for (auto&& callback : callback_function_list_)
        multi_thread_executor_->EnqueueTask(callback, message.value());
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
  Status status_{Status::kRun};
  const int max_queue_size_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread thread_for_callback_manager_;

  std::deque<DataType> message_queue_;
  std::list<std::function<void(DataType)>> callback_function_list_;

  std::unique_ptr<MultiThreadExecutor> multi_thread_executor_;
};

#endif  // MESSAGE_HANDLER_H_