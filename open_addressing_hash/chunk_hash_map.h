#ifndef CHUNK_HASH_MAP_H_
#define CHUNK_HASH_MAP_H_

#include <functional>
#include <unordered_map>
#include <vector>

#include "types.h"

template <typename ValueT>
class Chunk {
  static constexpr size_t kChunkSize{512};  // 8x8x8 청크
 public:
  void InitializeChunk() { data_.resize(kChunkSize); }
  ValueT& At(size_t index) { return data_[index]; }

 private:
  int id_{0};
  std::vector<ValueT> data_;
};

template <typename ValueType>
class ChunkHashMap {
 public:
  ChunkHashMap() { map_.reserve(2000000); }

  void insert(const Vec3i& key, const ValueType& value) {
    Vec3i coarse_index{(key.x + 32768) >> 3, (key.y + 32768) >> 3,
                       (key.z + 32768) >> 3};
    uint64_t coarse_morton_code = coarse_index.GetMortonCode();
    if (map_.find(coarse_morton_code) == map_.end()) {
      map_[coarse_morton_code].InitializeChunk();
    }
    // 청크 내의 인덱스에 접근
    Vec3i local_index{(key.x + 32768) % 8, (key.y + 32768) % 8,
                      (key.z + 32768) % 8};
    map_[coarse_morton_code].At(local_index.GetMortonCode()) = value;
  }

  ValueType& at(const Vec3i& key) {
    Vec3i coarse_index{(key.x + 32768) >> 3, (key.y + 32768) >> 3,
                       (key.z + 32768) >> 3};
    uint64_t coarse_morton_code = coarse_index.GetMortonCode();
    auto it = map_.find(coarse_morton_code);
    if (it == map_.end()) {
      throw std::out_of_range("Key not found in ChunkHashMap");
    }
    Vec3i local_index{(key.x + 32768) % 8, (key.y + 32768) % 8,
                      (key.z + 32768) % 8};
    return it->second.At(local_index.GetMortonCode());
  }

  size_t size() const { return map_.size(); }

 private:
  std::unordered_map<uint64_t, Chunk<ValueType>> map_;
};

#endif  // CHUNK_HASH_MAP_H_
