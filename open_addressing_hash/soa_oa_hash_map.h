#ifndef OPEN_ADDRESSING_HASH_SOA_OPEN_ADDRESSING_HASH_MAP_H_
#define OPEN_ADDRESSING_HASH_SOA_OPEN_ADDRESSING_HASH_MAP_H_

#include <functional>

template <typename KeyType, typename ValueType>
struct SOAHashEntry {
  KeyType* key{nullptr};
  ValueType* value{nullptr};
  uint64_t* hash_value{nullptr};
  bool* is_occupied{nullptr};
  bool* is_deleted{nullptr};  // 삭제 표시 추가

  SOAHashEntry(const size_t capacity) {
    key = new KeyType[capacity];
    value = new ValueType[capacity];
    hash_value = new uint64_t[capacity];
    is_occupied = new bool[capacity]();
    is_deleted = new bool[capacity]();
    std::fill_n(is_occupied, capacity, false);
    std::fill_n(is_deleted, capacity, false);
  }

  ~SOAHashEntry() {
    delete[] key;
    delete[] value;
    delete[] hash_value;
    delete[] is_occupied;
    delete[] is_deleted;
  }
};

template <typename KeyType, typename ValueType, typename HashFunction>
class OpenAddressingHash {
 public:
  OpenAddressingHash(size_t size)
      : num_activated_slot_(0),
        num_slot_(FindNextPowerOfTwo(size)),
        slots_(num_slot_) {}

  ~OpenAddressingHash() {}

  // Iterator 클래스 정의
  class iterator {
   public:
    using value_type = std::pair<const KeyType&, ValueType&>;
    using reference = value_type;
    using pointer = value_type*;

    iterator(OpenAddressingHash* hash_map, size_t index)
        : hash_map_(hash_map), index_(index) {
      // 생성 시 유효한 위치로 이동
      if (index_ < hash_map_->num_slot_ &&
          (!hash_map_->slots_.is_occupied[index_] ||
           hash_map_->slots_.is_deleted[index_])) {
        advance_to_valid();
      }
    }

    // 전위 증가 연산자
    iterator& operator++() {
      if (index_ < hash_map_->num_slot_) {
        ++index_;
        advance_to_valid();
      }
      return *this;
    }

    // 후위 증가 연산자
    iterator operator++(int) {
      iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    // 비교 연산자
    bool operator==(const iterator& other) const {
      return hash_map_ == other.hash_map_ && index_ == other.index_;
    }

    bool operator!=(const iterator& other) const { return !(*this == other); }

    // 역참조 연산자
    value_type operator*() const {
      return {hash_map_->slots_.key[index_], hash_map_->slots_.value[index_]};
    }

    // 화살표 연산자 추가
    value_type* operator->() const {
      static value_type result = {hash_map_->slots_.key[index_],
                                  hash_map_->slots_.value[index_]};
      return &result;
    }

    // index_ 접근자 (Erase 메서드에서 사용)
    size_t GetIndex() const { return index_; }

   private:
    // 다음 유효한 엔트리로 이동 - 삭제된 슬롯도 건너뜀
    void advance_to_valid() {
      while (index_ < hash_map_->num_slot_ &&
             (!hash_map_->slots_.is_occupied[index_] ||
              hash_map_->slots_.is_deleted[index_])) {
        ++index_;
      }
    }

    OpenAddressingHash* hash_map_;
    size_t index_;

    friend class OpenAddressingHash;
  };

  // const_iterator 클래스 정의
  class const_iterator {
   public:
    using value_type = std::pair<const KeyType&, const ValueType&>;

    const_iterator(const OpenAddressingHash* hash_map, size_t index)
        : hash_map_(hash_map), index_(index) {
      if (index_ < hash_map_->num_slot_ &&
          (!hash_map_->slots_.is_occupied[index_] ||
           hash_map_->slots_.is_deleted[index_])) {
        MoveToValidIndex();
      }
    }

    // 전위 증가 연산자
    const_iterator& operator++() {
      if (index_ < hash_map_->num_slot_) {
        ++index_;
        MoveToValidIndex();
      }
      return *this;
    }

    // 후위 증가 연산자
    const_iterator operator++(int) {
      const_iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    // 비교 연산자
    bool operator==(const const_iterator& other) const {
      return hash_map_ == other.hash_map_ && index_ == other.index_;
    }

    bool operator!=(const const_iterator& other) const {
      return !(*this == other);
    }

    // 역참조 연산자
    value_type operator*() const {
      return {hash_map_->slots_.key[index_], hash_map_->slots_.value[index_]};
    }

    // 화살표 연산자 추가
    value_type* operator->() const {
      static value_type result = {hash_map_->slots_.key[index_],
                                  hash_map_->slots_.value[index_]};
      return &result;
    }

   private:
    // 다음 유효한 엔트리로 이동 - 삭제된 슬롯도 건너뜀
    void MoveToValidIndex() {
      while (index_ < hash_map_->num_slot_ &&
             (!hash_map_->slots_.is_occupied[index_] ||
              hash_map_->slots_.is_deleted[index_])) {
        ++index_;
      }
    }

    const OpenAddressingHash* hash_map_;
    size_t index_;
  };

  iterator begin() { return iterator(this, 0); }
  iterator end() { return iterator(this, num_slot_); }
  const_iterator begin() const { return const_iterator(this, 0); }
  const_iterator end() const { return const_iterator(this, num_slot_); }
  const_iterator cbegin() const { return const_iterator(this, 0); }
  const_iterator cend() const { return const_iterator(this, num_slot_); }

  bool insert(const KeyType& key, const ValueType& value) {
    // 로드 팩터 체크 (삭제된 슬롯도 포함)
    if (num_activated_slot_ >= num_slot_ * load_factor_) resize(num_slot_ * 2);

    const auto hash_value = hash_function_(key);

    size_t index = hash_value & (num_slot_ - 1);  // 비트 마스킹
    size_t original_index = index;
    size_t i = 0;                      // 이차 탐사용 카운터
    size_t first_deleted = num_slot_;  // 첫 번째 삭제된 슬롯 위치

    // 삽입 위치 찾기 - do-while 문을 while 문으로 변경
    while (i < num_slot_) {  // 모든 슬롯을 확인
      // 빈 슬롯을 찾은 경우
      if (!slots_.is_occupied[index]) {
        // 삭제된 슬롯이 있으면 그 위치를 사용
        if (first_deleted != num_slot_) {
          index = first_deleted;
        }
        // 슬롯에 데이터 삽입
        slots_.key[index] = key;
        slots_.value[index] = value;
        slots_.hash_value[index] = hash_value;
        slots_.is_occupied[index] = true;
        slots_.is_deleted[index] = false;
        ++num_activated_slot_;
        return true;
      } else if (slots_.is_deleted[index]) {
        // 삭제된 슬롯을 찾은 경우 기록
        if (first_deleted == num_slot_) first_deleted = index;
      } else if (slots_.key[index] == key) {
        // 이미 동일한 키가 존재하면 덮어쓰기
        slots_.value[index] = value;
        slots_.hash_value[index] = hash_value;
        slots_.is_deleted[index] = false;
        return true;
      }

      // 이차 탐사: 다음 위치 계산
      ++i;
      index = (original_index + i + i * i) & (num_slot_ - 1);
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
      // 빈 슬롯을 만나면 검색 중단 (삭제된 슬롯은 제외)
      if (!slots_.is_occupied[index] && !slots_.is_deleted[index]) {
        break;
      }
      // 점유된 슬롯이고 키가 일치하는 경우
      if (slots_.is_occupied[index] && !slots_.is_deleted[index] &&
          slots_.key[index] == key) {
        return iterator(this, index);
      }

      // 이차 탐사: 다음 위치 계산
      i++;
      index = (original_index + i + i * i) & (num_slot_ - 1);
    }

    return end();
  }

  const_iterator find(const KeyType& key) const {
    const auto hash_value = hash_function_(key);
    size_t index = hash_value & (num_slot_ - 1);  // 비트 마스킹
    size_t original_index = index;
    size_t i = 0;

    while (i < num_slot_) {
      // 빈 슬롯을 만나면 검색 중단 (삭제된 슬롯은 제외)
      if (!slots_.is_occupied[index] && !slots_.is_deleted[index]) {
        break;
      }
      // 점유된 슬롯이고 키가 일치하는 경우
      if (slots_.is_occupied[index] && !slots_.is_deleted[index] &&
          slots_.key[index] == key) {
        return const_iterator(this, index);
      }

      // 이차 탐사: 다음 위치 계산
      ++i;
      index = (original_index + i + i * i) & (num_slot_ - 1);
    }

    return end();
  }

  const ValueType& at(const KeyType& key) const {
    auto it = find(key);
    if (it == end()) throw std::out_of_range("Key not found");
    return (*it).second;
  }

  void erase(const iterator& it) {
    if (it != end()) {
      size_t index = it.GetIndex();
      if (slots_.is_occupied[index] && !slots_.is_deleted[index]) {
        slots_.is_deleted[index] = true;
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
      if (slots_.is_occupied[i] && !slots_.is_deleted[i]) {
        // 새 테이블에 해시 값 계산
        size_t index =
            slots_.hash_value[i] & (new_num_slot - 1);  // 비트 마스킹
        size_t original_index = index;
        size_t j = 0;  // 이차 탐사용 카운터

        // 빈 슬롯 찾기
        while (new_slots.is_occupied[index]) {
          j++;
          index = (original_index + j + j * j) & (new_num_slot - 1);

          // 이 경우는 발생하지 않아야 함 (new_num_slot이 충분히 큼)
          if (j >= new_num_slot)
            throw std::runtime_error("Resize failed: new table is full");
        }

        // 빈 슬롯에 데이터 복사 (복사 최적화)
        new_slots.key[index] = std::move(slots_.key[i]);
        new_slots.value[index] = std::move(slots_.value[i]);
        new_slots.hash_value[index] = std::move(slots_.hash_value[i]);
        new_slots.is_occupied[index] = true;
        ++new_activated;
      }
    }

    num_activated_slot_ = new_activated;
    num_slot_ = new_num_slot;
    slots_ = new_slots;
  }

  size_t num_activated_slot_{0};
  size_t num_slot_{0};
  SOAHashEntry<KeyType, ValueType> slots_;
  double load_factor_{0.7};
  HashFunction hash_function_;
};

#endif  // OPEN_ADDRESSING_HASH_OPEN_ADDRESSING_HASH_MAP_H_
