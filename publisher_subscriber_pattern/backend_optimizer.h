#ifndef BACKEND_OPTIMIZER_H_
#define BACKEND_OPTIMIZER_H_

#include <functional>
#include <iostream>

#include "publisher_subscriber.h"
#include "types.h"

class BackendOptimizer {
 public:
  BackendOptimizer() {
    std::vector<Submap> submap_list;
    Subscriber<std::vector<Submap>> submap_subscriber;
    submap_subscriber.Subscribe("/frontend_supplier/submap_list",
                                std::bind(&BackendOptimizer::CallbackSubmapList,
                                          this, std::placeholders::_1));
  }

 private:
  void CallbackSubmapList(
      std::shared_ptr<std::vector<Submap>> received_submap_list) {
    std::cerr << "CALLBACK!: " << received_submap_list->size() << std::endl;
  }
};

#endif  // TRAJECTORY_OPTIMIZER_H_
