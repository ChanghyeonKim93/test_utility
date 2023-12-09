#include <iostream>
#include <sstream>
#include <vector>

#include "message_handler.h"
#include "types.h"

using SubmapList = std::vector<Submap>;
using ScanList = std::vector<Scan>;

class FrontendSupplier {
 public:
  FrontendSupplier(const std::shared_ptr<MessageHandler<SubmapList>>&
                       msg_handler_submap_list,
                   const std::shared_ptr<MessageHandler<SubmapList>>&
                       msg_handler_optimized_submap_list)
      : msg_handler_submap_list_(msg_handler_submap_list),
        msg_handler_optimized_submap_list_(msg_handler_optimized_submap_list) {
    // Subscribe
    msg_handler_optimized_submap_list_->Subscribe(
        std::bind(&FrontendSupplier::CallbackOptimizedSubmapList, this,
                  std::placeholders::_1));
    th_ = std::thread([this]() { RunSubmapSupplierWhileLoop(); });
  }
  ~FrontendSupplier() { th_.join(); }

 private:
  void UpdateOptimizedSubmapList(const SubmapList& optimized_submap_list) {
    if (optimized_submap_list.empty()) return;

    const size_t num_submaps = optimized_submap_list.size();
    for (size_t index = 0; index < num_submaps; ++index) {
      const auto& optimized_submap = optimized_submap_list[index];

      std::lock_guard<std::mutex> lock(mutex_for_index_to_submap_);
      if (critical_data.index_to_submap_.find(optimized_submap.index) ==
          critical_data.index_to_submap_.end())
        continue;
      critical_data.index_to_submap_.at(optimized_submap.index).version =
          optimized_submap.version;
      critical_data.index_to_submap_.at(optimized_submap.index).pose =
          optimized_submap.pose;
    }
  }

  void CallbackOptimizedSubmapList(const SubmapList& optimized_submap_list) {
    // std::stringstream ss;
    // ss << "ThreadId[" << std::this_thread::get_id() << "] ";
    // ss << "FRONTEND gets optimized submap list from BACKEND.\n";
    // std::cerr << ss.str();
    UpdateOptimizedSubmapList(optimized_submap_list);
  }

  void RunSubmapSupplierWhileLoop() {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      Submap new_submap;
      new_submap.index = submap_index_++;
      new_submap.version = 0;
      new_submap.scan_list = {};

      SubmapList submap_list;
      {
        std::lock_guard<std::mutex> lock(mutex_for_index_to_submap_);
        critical_data.index_to_submap_.insert({new_submap.index, new_submap});
        submap_list.reserve(critical_data.index_to_submap_.size());
        for (const auto& [index, submap] : critical_data.index_to_submap_)
          submap_list.push_back(submap);
      }

      std::stringstream ss;
      ss << "ThreadId[" << std::this_thread::get_id() << "] ";
      ss << "FRONTEND publish!\n";
      std::cerr << ss.str();
      msg_handler_submap_list_->Publish(submap_list);
    }
  }

 private:
  std::thread th_;
  std::shared_ptr<MessageHandler<SubmapList>> msg_handler_submap_list_;
  std::shared_ptr<MessageHandler<SubmapList>>
      msg_handler_optimized_submap_list_;

  std::mutex mutex_for_index_to_submap_;

  int submap_index_{0};

  struct {
    std::unordered_map<int, Submap> index_to_submap_;
  } critical_data;
};

class BackendOptimizer {
 public:
  BackendOptimizer(const std::shared_ptr<MessageHandler<SubmapList>>&
                       msg_handler_submap_list,
                   const std::shared_ptr<MessageHandler<SubmapList>>&
                       msg_handler_optimized_submap_list)
      : msg_handler_submap_list_(msg_handler_submap_list),
        msg_handler_optimized_submap_list_(msg_handler_optimized_submap_list) {
    // Subscribe
    msg_handler_submap_list_->Subscribe(std::bind(
        &BackendOptimizer::CallbackSubmapList, this, std::placeholders::_1));

    thread_ = std::thread([this]() { RunSubmapOptimizerWhileLoop(); });
  }
  ~BackendOptimizer() { thread_.join(); }

 private:
  void RunSubmapOptimizerWhileLoop() {
    std::mutex mutex_for_cv;
    while (true) {
      std::unique_lock<std::mutex> lock(mutex_for_cv);
      cv_.wait(lock, [this]() { return true; });

      std::stringstream ss;
      ss << "Backend woke up! Optimization starts...\n";
      std::cerr << ss.str();

      OptimizeSubmapList();
      ss.str("");
      ss << "Backend optimization is done.\n";
      std::cerr << ss.str();

      SubmapList optimized_submap_list;
      {
        std::lock_guard<std::mutex> lock(mutex_for_index_to_submap_);
        optimized_submap_list.reserve(critical_data.index_to_submap_.size());
        for (const auto& [index, submap] : critical_data.index_to_submap_)
          optimized_submap_list.push_back(submap);
      }
      msg_handler_optimized_submap_list_->Publish(optimized_submap_list);
      ss.str("");
      ss << "Backend publish!\n";
      std::cerr << ss.str();
    }
  }

  void StoreIncomingSubmapList(const SubmapList& submap_list) {
    if (submap_list.empty()) return;

    const size_t num_submaps = submap_list.size();
    for (size_t index = 0; index < num_submaps; ++index) {
      const auto& submap = submap_list[index];

      std::lock_guard<std::mutex> lock(mutex_for_index_to_submap_);
      if (critical_data.index_to_submap_.find(submap.index) ==
          critical_data.index_to_submap_.end()) {
        critical_data.index_to_submap_.insert({submap.index, submap});
      } else {
        critical_data.index_to_submap_.at(submap.index).version =
            submap.version;
        critical_data.index_to_submap_.at(submap.index).pose = submap.pose;
      }
    }
  }

  void CallbackSubmapList(const SubmapList& submap_list) {
    // std::stringstream ss;
    // ss << "ThreadId[" << std::this_thread::get_id() << "] ";
    // ss << "Backend gets new submap list from Frontend.\n";
    // std::cerr << ss.str();
    StoreIncomingSubmapList(submap_list);
  }

  void OptimizeSubmapList() {
    mutex_for_index_to_submap_.lock();
    auto index_to_submap_to_be_optimized = critical_data.index_to_submap_;
    mutex_for_index_to_submap_.unlock();
    for (auto& [index, submap] : index_to_submap_to_be_optimized) {
      ++submap.version;
      submap.pose.x += 0.01;
      submap.pose.y += 0.002;
      submap.pose.theta -= 0.001;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(357));

    {
      std::lock_guard<std::mutex> lock(mutex_for_index_to_submap_);
      for (auto& [index, submap] : index_to_submap_to_be_optimized)
        critical_data.index_to_submap_.at(index) = submap;
    }
  }

 private:
  std::thread thread_;
  std::condition_variable cv_;

  std::mutex mutex_for_index_to_submap_;

  std::shared_ptr<MessageHandler<SubmapList>> msg_handler_submap_list_;
  std::shared_ptr<MessageHandler<SubmapList>>
      msg_handler_optimized_submap_list_;

  int submap_index_{0};
  struct {
    std::unordered_map<int, Submap> index_to_submap_;
  } critical_data;
};

int main() {
  try {
    // Imagine that this process is a ROS2 node.
    static constexpr int kMaxQueueSize{10};
    std::shared_ptr<MessageHandler<SubmapList>> msg_handler_submap_list =
        std::make_shared<MessageHandler<SubmapList>>(kMaxQueueSize);
    std::shared_ptr<MessageHandler<SubmapList>>
        msg_handler_optimized_submap_list =
            std::make_shared<MessageHandler<SubmapList>>(kMaxQueueSize);

    FrontendSupplier frontend(msg_handler_submap_list,
                              msg_handler_optimized_submap_list);
    BackendOptimizer backend(msg_handler_submap_list,
                             msg_handler_optimized_submap_list);
  } catch (std::exception& e) {
    std::cerr << "Error is catched: " << e.what() << std::endl;
  }
  return 0;
}
