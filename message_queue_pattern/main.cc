#include <iostream>
#include <sstream>
#include <vector>

#include "backend_optimizer.h"
#include "message_handler.h"
#include "types.h"

using namespace std::chrono_literals;

// template <class DataType>
// class MessageBroker {
//  public:
//   MessageBroker() {}
//   MessageBroker(const MessageBroker& rhs) = delete;
//   ~MessageBroker() {}
//   int GetId() const { return id_; }
//   void AddPoint() { ++id_; }

//  protected:
//   int id_{0};
// };

// template <typename DataType>
// class Publisher {
//  public:
//   Publisher() {
//     std::cerr << "broker id :" << typeid(DataType).name() << std::endl;
//     message_broker_.AddPoint();
//     std::cerr << message_broker_.GetId() << std::endl;
//   }

//  public:
//   static MessageBroker<DataType> message_broker_;
// };
// template <typename DataType>
// MessageBroker<DataType> Publisher<DataType>::message_broker_;

void CallbackFunction1(const std::vector<Submap>& submap_list) {
  std::stringstream ss;
  ss << "Callback function 1, size: " << submap_list.size() << "\n";
  std::cerr << ss.str();
}

void CallbackFunction2(const std::vector<Submap>& submap_list) {
  std::stringstream ss;
  ss << "Callback function 2, size: " << submap_list.size() << "\n";
  std::cerr << ss.str();
}

void CallbackFunction3(const std::vector<Submap>& submap_list) {
  std::stringstream ss;
  ss << "Callback function 3, size: " << submap_list.size() << "\n";
  std::cerr << ss.str();
}

int main() {
  std::cerr << "starts\n";

  // using SubmapListMessage = std::vector<Submap>;
  // using ScanListMessage = std::vector<Scan>;
  // Publisher<SubmapListMessage> a;
  // Publisher<SubmapListMessage> a2;
  // Publisher<SubmapListMessage> a3;
  // Publisher<ScanListMessage> b;
  // Publisher<ScanListMessage> b2;
  // Publisher<ScanListMessage> b3;

  // BackendOptimizer optimizer;
  constexpr int kMaxQueueSize{10};
  std::shared_ptr<MessageHandler<std::vector<Submap>>> message_queue =
      std::make_shared<MessageHandler<std::vector<Submap>>>(kMaxQueueSize);

  message_queue->Subscribe(
      std::bind(&CallbackFunction1, std::placeholders::_1));
  message_queue->Subscribe(
      std::bind(&CallbackFunction2, std::placeholders::_1));
  message_queue->Subscribe(
      std::bind(&CallbackFunction3, std::placeholders::_1));

  std::vector<Submap> submap_list;
  while (true) {
    std::this_thread::sleep_for(200ms);
    submap_list.push_back({});
    message_queue->Publish(submap_list);
  }

  return 0;
}