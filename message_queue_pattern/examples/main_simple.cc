#include <iostream>
#include <sstream>
#include <vector>

#include "backend_optimizer.h"
#include "message_handler.h"
#include "types.h"

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
    std::this_thread::sleep_for(std::chrono::microseconds(200));
    submap_list.push_back({});
    message_queue->Publish(submap_list);
  }

  return 0;
}
