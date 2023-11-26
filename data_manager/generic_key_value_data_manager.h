#ifndef GENERIC_KEY_VALUE_DATA_MANAGER_H_
#define GENERIC_KEY_VALUE_DATA_MANAGER_H_

#include <iostream>
#include <memory>
#include <mutex>
#include <typeindex>
#include <unordered_map>

class GenericKeyValueDataManager {
 public:
  GenericKeyValueDataManager() {}
  ~GenericKeyValueDataManager() {
    for (auto& [data_type_info, ptr] : data_map_ptrs_for_each_types_) {
      delete ptr;
    }
  }

  template <typename _DataType>
  void AddData(const int key, const _DataType& data) {
    using DataMap = std::unordered_map<int, _DataType>;
    std::lock_guard<std::mutex> local_lock_guard(mutex_);

    if (data_map_ptrs_for_each_types_.find(typeid(_DataType)) !=
        data_map_ptrs_for_each_types_.end()) {
      reinterpret_cast<DataMap*>(
          data_map_ptrs_for_each_types_.at(typeid(_DataType)))
          ->insert({key, data});
    } else {
      DataMap* data_map_ptr = new DataMap();
      data_map_ptrs_for_each_types_.insert(
          {typeid(_DataType), reinterpret_cast<void*>(data_map_ptr)});
      data_map_ptr->insert({key, data});
    }
  }

  template <typename _DataType>
  _DataType GetData(const int key) {
    using DataMap = std::unordered_map<int, _DataType>;
    std::lock_guard<std::mutex> local_lock_guard(mutex_);

    if (data_map_ptrs_for_each_types_.find(typeid(_DataType)) !=
        data_map_ptrs_for_each_types_.end()) {
      DataMap* data_map_ptr = reinterpret_cast<DataMap*>(
          data_map_ptrs_for_each_types_.at(typeid(_DataType)));
      if (data_map_ptr->find(key) != data_map_ptr->end())
        return data_map_ptr->at(key);
      else
        throw std::runtime_error("No data in map.\n");
    } else {
      throw std::runtime_error("There is no type registered.\n");
    }
  }

 private:
  std::mutex mutex_;

  std::unordered_map<std::type_index, void*> data_map_ptrs_for_each_types_;
};

#endif  // GENERIC_KEY_VALUE_DATA_MANAGER_H_