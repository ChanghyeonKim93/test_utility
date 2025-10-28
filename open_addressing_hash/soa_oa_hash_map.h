#ifndef OPEN_ADDRESSING_HASH_SOA_OPEN_ADDRESSING_HASH_MAP_H_
#define OPEN_ADDRESSING_HASH_SOA_OPEN_ADDRESSING_HASH_MAP_H_

#include <functional>

template <typename KeyType, typename ValueType>
struct SOAHashEntry {
  KeyType* key{nullptr};
  ValueType* value{nullptr};
  uint64_t* hash_value{nullptr};
  uint8_t* state{nullptr};

  SOAHashEntry(const size_t capacity) {
    key = new KeyType[capacity];
    value = new ValueType[capacity];
    hash_value = new uint64_t[capacity];
    state = new uint8_t[capacity]();
    std::fill_n(state, capacity, 0b00);
  }

  ~SOAHashEntry() {
    delete[] key;
    delete[] value;
    delete[] hash_value;
    delete[] state;
  }

  inline void SetOccupied(size_t index) { state[index] |= 0b01; }
  inline void SetDeleted(size_t index) { state[index] |= 0b10; }
  inline void UnsetOccupied(size_t index) { state[index] &= 0b10; }
  inline void UnsetDeleted(size_t index) { state[index] &= 0b01; }
  inline bool IsOccupied(size_t index) const { return state[index] & 0b01; }
  inline bool IsEmpty(size_t index) const { return !(state[index] & 0b01); }
  inline bool IsDeleted(size_t index) const { return state[index] & 0b10; }
  inline bool IsNeverUsed(size_t index) const {
    return !(state[index] & 0b01) && !(state[index] & 0b10);
  }
};

template <typename KeyType, typename ValueType,
          typename HashFunction = std::hash<KeyType>>
class OpenAddressingHash {
  using KeyValuePair = std::pair<const KeyType&, const ValueType&>;

 public:
  OpenAddressingHash(size_t size)
      : num_activated_slot_(0),
        num_slot_(FindNextPowerOfTwo(size)),
        data_(num_slot_) {}

  ~OpenAddressingHash() {}

  // Iterator 클래스 정의
  class iterator {
   public:
    iterator(OpenAddressingHash* hash_map, size_t index)
        : hash_map_(hash_map), index_(index) {
      if (index_ < hash_map_->num_slot_ &&
          (!hash_map_->data_.IsOccupied(index_) ||
           hash_map_->data_.IsDeleted(index_)))
        MoveToNextValidIndex();
    }

    iterator& operator++() {
      if (index_ < hash_map_->num_slot_) {
        ++index_;
        MoveToNextValidIndex();
      }
      return *this;
    }

    iterator operator++(int) {
      iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    bool operator==(const iterator& other) const {
      return hash_map_ == other.hash_map_ && index_ == other.index_;
    }

    bool operator!=(const iterator& other) const { return !(*this == other); }

    KeyValuePair operator*() const {
      return {hash_map_->data_.key[index_], hash_map_->data_.value[index_]};
    }

    KeyValuePair* operator->() const {
      static KeyValuePair result = {hash_map_->data_.key[index_],
                                    hash_map_->data_.value[index_]};
      return &result;
    }

    size_t GetIndex() const { return index_; }

   private:
    void MoveToNextValidIndex() {
      while (index_ < hash_map_->num_slot_ &&
             (hash_map_->data_.IsEmpty(index_) ||
              hash_map_->data_.IsDeleted(index_))) {
        ++index_;
      }
    }

    OpenAddressingHash* hash_map_{nullptr};
    size_t index_{0};
  };

  class const_iterator {
   public:
    const_iterator(const OpenAddressingHash* hash_map, size_t index)
        : hash_map_(hash_map), index_(index) {
      if (index_ < hash_map_->num_slot_ &&
          (hash_map_->data_.IsEmpty(index_) ||
           hash_map_->data_.IsDeleted(index_))) {
        MoveToValidIndex();
      }
    }

    const_iterator& operator++() {
      if (index_ < hash_map_->num_slot_) {
        ++index_;
        MoveToValidIndex();
      }
      return *this;
    }

    const_iterator operator++(int) {
      const_iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    bool operator==(const const_iterator& rhs) const {
      return hash_map_ == rhs.hash_map_ && index_ == rhs.index_;
    }

    bool operator!=(const const_iterator& rhs) const { return !(*this == rhs); }

    KeyValuePair operator*() const {
      return {hash_map_->data_.key[index_], hash_map_->data_.value[index_]};
    }

    KeyValuePair* operator->() const {
      static KeyValuePair result = {hash_map_->data_.key[index_],
                                    hash_map_->data_.value[index_]};
      return &result;
    }

   private:
    void MoveToValidIndex() {
      while (index_ < hash_map_->num_slot_ &&
             (hash_map_->data_.IsEmpty(index_) ||
              hash_map_->data_.IsDeleted(index_))) {
        ++index_;
      }
    }

    const OpenAddressingHash* hash_map_{nullptr};
    size_t index_{0};
  };

  iterator begin() { return iterator(this, 0); }
  iterator end() { return iterator(this, num_slot_); }
  const_iterator begin() const { return const_iterator(this, 0); }
  const_iterator end() const { return const_iterator(this, num_slot_); }
  const_iterator cbegin() const { return const_iterator(this, 0); }
  const_iterator cend() const { return const_iterator(this, num_slot_); }

  bool insert(const KeyType& key, const ValueType& value) {
    // Check load factor and resize if necessary
    if (num_activated_slot_ >= num_slot_ * load_factor_) resize(num_slot_ * 2);

    // `operator()` of hash function should be defined
    const auto hash_value = hash_function_(key);

    size_t index = probe(hash_value, 0);
    size_t original_index = index;
    size_t i = 0;
    while (i < num_slot_) {
      // If empty slot found
      if (data_.IsEmpty(index) || data_.IsDeleted(index)) {
        // Insert new key-value pair
        if (data_.key[index] != key || data_.IsDeleted(index))
          ++num_activated_slot_;
        data_.key[index] = key;
        data_.value[index] = value;
        data_.hash_value[index] = hash_value;
        data_.SetOccupied(index);
        data_.UnsetDeleted(index);
        return true;
      }
      // Quadratic probing
      index = probe(original_index, ++i);
    }

    // 여기까지 왔다면 테이블이 가득 찼거나 모든 슬롯이 점유됨
    resize(num_slot_ * 2);
    return insert(key, value);  // 확장된 테이블에 다시 삽입 시도
  }

  iterator find(const KeyType& key) {
    size_t index = hash_function_(key) & (num_slot_ - 1);  // 비트 마스킹
    size_t original_index = index;
    size_t i = 0;
    while (i < num_slot_) {
      // Stop search if an empty slot is found (excluding deleted slots)
      if (data_.IsNeverUsed(index)) break;
      // If occupied slot and key matches
      if (data_.IsOccupied(index) && data_.key[index] == key)
        return iterator(this, index);
      index = probe(original_index, ++i);
    }
    return end();
  }

  const_iterator find(const KeyType& key) const {
    const auto hash_value = hash_function_(key);
    size_t index = hash_value & (num_slot_ - 1);  // 비트 마스킹
    size_t original_index = index;
    size_t i = 0;
    while (i < num_slot_) {
      // Stop search if an empty slot is found (excluding deleted slots)
      if (data_.IsNeverUsed(index)) break;
      // If occupied slot and key matches
      if (data_.IsOccupied(index) && data_.key[index] == key)
        return const_iterator(this, index);
      // Quadratic probing
      index = probe(original_index, ++i);
    }

    return end();
  }

  const ValueType& at(const KeyType& key) const {
    auto it = find(key);
    if (it == end()) throw std::out_of_range("Key is not found");
    return (*it).second;
  }

  void erase(const iterator& it) {
    if (it != end()) {
      size_t index = it.GetIndex();
      if (data_.IsOccupied(index)) {
        data_.UnsetOccupied(index);
        data_.SetDeleted(index);
        --num_activated_slot_;
      }
    }
  }

  void erase(const KeyType& key) { erase(find(key)); }

  size_t size() const { return num_activated_slot_; }

  bool empty() const { return num_activated_slot_ == 0; }

  void set_load_factor(const double load_factor) {
    if (load_factor <= 0.0 || load_factor >= 1.0)
      throw std::invalid_argument("Load factor must be between 0 and 1");
    load_factor_ = load_factor;
  }

  double get_load_factor() const {
    return static_cast<double>(num_activated_slot_) / num_slot_;
  }

 private:
  inline size_t probe(const size_t hash_index, const size_t probe_count) {
    return (hash_index + probe_count * probe_count) & (num_slot_ - 1);
  }

  size_t FindNextPowerOfTwo(size_t n) {
    size_t power = 1;
    while (power < n) power *= 2;
    return power;
  }

  void resize(const size_t new_size) {
    size_t new_num_slot = FindNextPowerOfTwo(new_size);
    SOAHashEntry<KeyType, ValueType> new_slots(new_num_slot);

    size_t new_activated = 0;
    for (size_t i = 0; i < num_slot_; ++i) {
      if (data_.IsOccupied(i)) {
        // 새 테이블에 해시 값 계산
        size_t index = data_.hash_value[i] & (new_num_slot - 1);  // 비트 마스킹
        size_t original_index = index;
        size_t j = 0;  // 이차 탐사용 카운터

        // 빈 슬롯 찾기
        while (new_slots.IsOccupied(index)) {
          j++;
          index = (original_index + j * j) & (new_num_slot - 1);

          // 이 경우는 발생하지 않아야 함 (new_num_slot이 충분히 큼)
          if (j >= new_num_slot)
            throw std::runtime_error("Resize failed: new table is full");
        }

        // 빈 슬롯에 데이터 복사 (복사 최적화)
        new_slots.key[index] = std::move(data_.key[i]);
        new_slots.value[index] = std::move(data_.value[i]);
        new_slots.hash_value[index] = std::move(data_.hash_value[i]);
        new_slots.SetOccupied(index);
        ++new_activated;
      }
    }

    num_activated_slot_ = new_activated;
    num_slot_ = new_num_slot;
    data_ = new_slots;
  }

  size_t num_activated_slot_{0};
  size_t num_slot_{0};
  SOAHashEntry<KeyType, ValueType> data_;
  double load_factor_{0.7};
  HashFunction hash_function_;
};

#endif  // OPEN_ADDRESSING_HASH_OPEN_ADDRESSING_HASH_MAP_H_
