#ifndef MESSAGE_QUEUE_PATTERN_FRONTEND_SUPPLIER_H_
#define MESSAGE_QUEUE_PATTERN_FRONTEND_SUPPLIER_H_

#include <functional>
#include <mutex>
#include <thread>

template <typename MessageType>
class MessageHandler {
 public:
  MessageHandler() {}
  ~MessageHandler() {}

 private:
};

#endif  // MESSAGE_QUEUE_PATTERN_FRONTEND_SUPPLIER_H_
