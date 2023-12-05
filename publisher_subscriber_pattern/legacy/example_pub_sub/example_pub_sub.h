#pragma once

#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/* */
namespace messaging {

template <class DataType>
class MessageQueue {
 public:
  MessageQueue() {
    std::thread t = std::thread([this]() { this->RunCallbacksWhileLoop(); });
    t.detach();
  }

  void EnqueueMessage(std::shared_ptr<DataType const> message) {
    message_list_.push_back(message);
  }

  template <typename CallbackFunction>
  void RegisterCallback(CallbackFunction&& callback_function) {
    std::lock_guard lock(callback_function_mutex_);
    callback_functions_.template emplace_back(
        std::forward<CallbackFunction>(callback));
  }

  void RunCallbacksWhileLoop() {
    while (true) {
      std::chrono::time_point<std::chrono::system_clock> now =
          std::chrono::system_clock::now();
      auto milliseconds =
          std::chrono::duration_cast<std::chrono::milliseconds>(prev - now);
      auto ms = 100 - milliseconds.count();
      while (ms > 0) {
        --ms;
      }
      if (!callback_functions_.empty() && !message_list_.empty()) {
        std::lock_guard lock(callback_function_mutex_);
        std::shared_ptr<DataType const> message = DequeueMessage();
        for (auto&& callback : callback_functions_) {
          // 각 콜백함수를 실행하고 온다.
          std::thread t = std::thread(callback, message);
          t.detach();
        }
      }
      prev = std::chrono::system_clock::now();
    }
  }

 private:
  std::shared_ptr<DataType const> DequeueMessage() {
    if (message_list_.empty()) return nullptr;

    const auto item = message_list_.front();
    message_list_.pop_front();
    return item;
  }

 private:
  std::mutex callback_function_mutex_;
  std::string topic_name_;
  std::deque<std::shared_ptr<DataType const>> message_list_;
  std::vector<std::function<void(std::shared_ptr<DataType const>)>>
      callback_functions_ = {};
  std::chrono::time_point<std::chrono::system_clock> prev =
      std::chrono::system_clock::now();
};

}  // namespace messaging
