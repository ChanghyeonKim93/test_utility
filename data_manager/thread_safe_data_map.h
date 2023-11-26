#ifndef THREAD_SAFE_DATA_MAP_H_
#define THREAD_SAFE_DATA_MAP_H_

#include <iostream>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

template <typename _DataType>
class ThreadSafeDataMap {
 public:
  ThreadSafeDataMap() { key_data_map_.reserve(100000); }

  void RegisterData(const int key, const _DataType& data) {
    std::lock_guard<std::mutex> local_lock_guard(mutex_);
    key_data_map_.insert({key, data});
  }

  void UpdateData(const int key, const _DataType& updated_data) {
    std::lock_guard<std::mutex> local_lock_guard(mutex_);
    if (key_data_map_.find(key) == key_data_map_.end())
      throw std::runtime_error("Key does not exist.");

    key_data_map_.at(key) = updated_data;
  }

  _DataType GetData(const int key) {
    std::lock_guard<std::mutex> local_lock_guard(mutex_);
    if (key_data_map_.find(key) == key_data_map_.end())
      throw std::runtime_error("Key does not exist.");

    return key_data_map_.at(key);
  }

 private:
  std::mutex mutex_;
  std::unordered_map<int, _DataType> key_data_map_;
};

#endif  // THREAD_SAFE_DATA_MAP_H_