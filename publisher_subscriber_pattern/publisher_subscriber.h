#ifndef PUBLISHER_SUBSCRIBER_H_
#define PUBLISHER_SUBSCRIBER_H_

// submap_subscriber.Subscribe<std::vector<Scan>>(std::bind(&BackendOptimizer::CallbackSubmapList,std::placeholders::_1)));
#include <condition_variable>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

template <typename DataType>
class TopicServer {
 public:
  void EnqueueMessage(const DataType& message) {}
  void DisconnectSubscriber() {}

 private:
  void ServiceLoop() {
    std::unique_lock<std::mutex> loop_lock(loop_mutex_);

    while (true) {
      if (0 == m_vecQueuedNotifications.size()) {
        m_cvPauseFlushThread.wait(loop_lock);
      }

      if (m_bShutdown) break;

      FlushQueuedNotifications();
    }
  }

 private:
  std::mutex loop_mutex_;
  std::mutex topic_mutex_;
  std::deque<std::function<void(int, int)>> function_queue_;

  // topic name to subscribers
  // topic name to publisher

  std::unordered_map<std::string,
                     std::unordered_set<std::shared_ptr<Subscriber<DataType>>>>
      topic_name_to_subscribers_;
  std::unordered_map<std::string, std::shared_ptr<Publisher<DataType>>>
      topic_name_to_publisher_;
};

template <typename DataType>
class Publisher {
 public:
  Publisher() {}
  ~Publisher() {}

  void Advertise(const std::string& topic_name) {}

  void Publish(const DataType& message) {
    // enqueue data

    // notify subscribers
  }

 protected:
  TopicServer<DataType> topic_server_;
};

template <typename DataType>
class Subscriber {
 public:
  void Subscribe(
      const std::string& topic_name_to_subscribe,
      std::function<void(std::shared_ptr<DataType>)>&& callback_function) {
    std::cerr << "Subscribed!\n";
    callback_function({});
  }

 private:
  void EnrollCallbackFunction() {}

 protected:
  TopicServer<DataType> topic_server_;
};

template <typename DataType>
class EventChannel {};

#endif  // PUBLISHER_SUBSCRIBER_H_