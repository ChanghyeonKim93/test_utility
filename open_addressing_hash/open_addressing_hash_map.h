#ifndef OPEN_ADDRESSING_HASH_OPEN_ADDRESSING_HASH_MAP_H_
#define OPEN_ADDRESSING_HASH_OPEN_ADDRESSING_HASH_MAP_H_

#include <functional>

template <typename KeyType, typename ValueType>
struct HashEntry {
  KeyType key;
  ValueType value;
  bool is_occupied{false};
  bool is_deleted{false};  // 삭제 표시 추가
};

template <typename KeyType, typename ValueType,
          typename HashFunction = std::hash<KeyType>>
class OpenAddressingHash {
 public:
  // 2의 제곱으로 올림 함수
  size_t FindNextPowerOfTwo(size_t n) {
    size_t power = 1;
    while (power < n) {
      power *= 2;
    }
    return power;
  }

  OpenAddressingHash(size_t size) : num_activated_slot_(0) {
    // 테이블 크기를 2의 제곱으로 설정
    num_slot_ = FindNextPowerOfTwo(size);
    slots_ = new HashEntry<KeyType, ValueType>[num_slot_];
  }

  ~OpenAddressingHash() { delete[] slots_; }

  // Iterator 클래스 정의
  class iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using KeyValuePair = std::pair<const KeyType&, ValueType&>;
    using difference_type = std::ptrdiff_t;
    using KeyValuePairPtr = KeyValuePair*;
    using KeyValuePairRef = KeyValuePair;

    iterator(OpenAddressingHash* hash, size_t index)
        : hash_map_(hash), index_(index) {
      // 생성 시 유효한 위치로 이동
      if (index_ < hash_map_->num_slot_ &&
          (!hash_map_->slots_[index_].is_occupied ||
           hash_map_->slots_[index_].is_deleted)) {
        MoveToNextValidIndex();
      }
    }

    // 전위 증가 연산자
    iterator& operator++() {
      if (index_ < hash_map_->num_slot_) {
        ++index_;
        MoveToNextValidIndex();
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
    KeyValuePairRef operator*() const {
      return {hash_map_->slots_[index_].key, hash_map_->slots_[index_].value};
    }

    // 화살표 연산자 추가
    KeyValuePairPtr operator->() const {
      static KeyValuePair result = {hash_map_->slots_[index_].key,
                                    hash_map_->slots_[index_].value};
      return &result;
    }

    // index_ 접근자 (Erase 메서드에서 사용)
    size_t get_index() const { return index_; }

   private:
    // 다음 유효한 엔트리로 이동 - 삭제된 슬롯도 건너뜀
    void MoveToNextValidIndex() {
      while (index_ < hash_map_->num_slot_ &&
             (!hash_map_->slots_[index_].is_occupied ||
              hash_map_->slots_[index_].is_deleted)) {
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
    using iterator_category = std::forward_iterator_tag;
    using KeyValuePair = std::pair<const KeyType&, const ValueType&>;
    using difference_type = std::ptrdiff_t;
    using pointer = KeyValuePair*;
    using reference = KeyValuePair;

    const_iterator(const OpenAddressingHash* hash, size_t index)
        : hash_map_(hash), index_(index) {
      // 생성 시 유효한 위치로 이동
      if (index_ < hash_map_->num_slot_ &&
          (!hash_map_->slots_[index_].is_occupied ||
           hash_map_->slots_[index_].is_deleted)) {
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
    reference operator*() const {
      return {hash_map_->slots_[index_].key, hash_map_->slots_[index_].value};
    }

    // 화살표 연산자 추가
    pointer operator->() const {
      static KeyValuePair result = {hash_map_->slots_[index_].key,
                                    hash_map_->slots_[index_].value};
      return &result;
    }

   private:
    // 다음 유효한 엔트리로 이동 - 삭제된 슬롯도 건너뜀
    void MoveToValidIndex() {
      while (index_ < hash_map_->num_slot_ &&
             (!hash_map_->slots_[index_].is_occupied ||
              hash_map_->slots_[index_].is_deleted)) {
        ++index_;
      }
    }

    const OpenAddressingHash* hash_map_;
    size_t index_;
  };

  // 반복자 관련 메서드
  iterator begin() { return iterator(this, 0); }

  iterator end() { return iterator(this, num_slot_); }

  const_iterator begin() const { return const_iterator(this, 0); }

  const_iterator end() const { return const_iterator(this, num_slot_); }

  const_iterator cbegin() const { return const_iterator(this, 0); }

  const_iterator cend() const { return const_iterator(this, num_slot_); }

  bool insert(const KeyType& key, const ValueType& value) {
    // 로드 팩터 체크 (삭제된 슬롯도 포함)
    if (num_activated_slot_ >= num_slot_ * load_factor_) {
      Resize(num_slot_ * 2);
    }

    const auto hash_value = hash_function_(key);
    size_t index = hash_value & (num_slot_ - 1);  // 비트 마스킹
    size_t original_index = index;
    size_t i = 0;                      // 이차 탐사용 카운터
    size_t first_deleted = num_slot_;  // 첫 번째 삭제된 슬롯 위치

    // 삽입 위치 찾기
    do {
      // 빈 슬롯을 찾은 경우
      if (!slots_[index].is_occupied) {
        // 삭제된 슬롯이 있으면 그 위치를 사용
        if (first_deleted != num_slot_) {
          index = first_deleted;
        }
        // 슬롯에 데이터 삽입
        slots_[index].key = key;
        slots_[index].value = value;
        slots_[index].is_occupied = true;
        slots_[index].is_deleted = false;
        ++num_activated_slot_;
        return true;
      }
      // 삭제된 슬롯을 찾은 경우 기록
      else if (slots_[index].is_deleted) {
        if (first_deleted == num_slot_) {
          first_deleted = index;
        }
      }
      // 이미 동일한 키가 존재하면 덮어쓰기
      else if (slots_[index].key == key) {
        slots_[index].value = value;
        slots_[index].is_deleted =
            false;  // 기존 슬롯이 삭제 표시되어 있을 수 있음
        return true;
      }

      // 이차 탐사: 다음 위치 계산
      i++;
      index = (original_index + i + i * i) & (num_slot_ - 1);
    } while (i < num_slot_);  // 모든 슬롯을 확인

    // 여기까지 왔다면 테이블이 가득 찼거나 모든 슬롯이 점유됨
    Resize(num_slot_ * 2);
    return insert(key, value);  // 확장된 테이블에 다시 삽입 시도
  }

  iterator find(const KeyType& key) {
    const auto hash_value = hash_function_(key);
    size_t index = hash_value & (num_slot_ - 1);  // 비트 마스킹
    size_t original_index = index;
    size_t i = 0;

    do {
      // 빈 슬롯을 만나면 검색 중단 (삭제된 슬롯은 제외)
      if (!slots_[index].is_occupied && !slots_[index].is_deleted) {
        break;
      }
      // 점유된 슬롯이고 키가 일치하는 경우
      if (slots_[index].is_occupied && !slots_[index].is_deleted &&
          slots_[index].key == key) {
        return iterator(this, index);
      }

      // 이차 탐사: 다음 위치 계산
      i++;
      index = (original_index + i + i * i) & (num_slot_ - 1);
    } while (i < num_slot_);

    return end();
  }

  const ValueType& At(const KeyType& key) {
    auto it = find(key);
    if (it == end()) {
      throw std::out_of_range("Key not found");
    }
    return (*it).second;
  }

  void erase(const KeyType& key) {
    auto it = find(key);
    if (it != end()) {
      size_t index = it.GetIndex();
      slots_[index].is_deleted = true;  // 삭제 표시만 함
      --num_activated_slot_;
    }
  }

  void erase(const iterator& it) {
    if (it != end()) {
      size_t index = it.get_index();
      if (slots_[index].is_occupied && !slots_[index].is_deleted) {
        slots_[index].is_deleted = true;
        --num_activated_slot_;
      }
    }
  }

  // 요소 개수 반환
  size_t size() const { return num_activated_slot_; }

  // 해시 테이블이 비었는지 확인
  bool empty() const { return num_activated_slot_ == 0; }

  // 로드 팩터 설정
  void set_load_factor(double factor) {
    if (factor <= 0.0 || factor >= 1.0) {
      throw std::invalid_argument("Load factor must be between 0 and 1");
    }
    load_factor_ = factor;
  }

  // 현재 로드 팩터 반환
  double get_load_factor() const {
    return static_cast<double>(num_activated_slot_) / num_slot_;
  }

 private:
  void Resize(const size_t new_size) {
    size_t new_num_slot = FindNextPowerOfTwo(new_size);
    HashEntry<KeyType, ValueType>* new_slots =
        new HashEntry<KeyType, ValueType>[new_num_slot];
    size_t new_activated = 0;

    for (size_t i = 0; i < num_slot_; ++i) {
      if (slots_[i].is_occupied && !slots_[i].is_deleted) {
        // 새 테이블에 해시 값 계산
        const auto hash_value = hash_function_(slots_[i].key);
        size_t index = hash_value & (new_num_slot - 1);  // 비트 마스킹
        size_t original_index = index;
        size_t j = 0;  // 이차 탐사용 카운터

        // 빈 슬롯 찾기
        while (new_slots[index].is_occupied) {
          j++;
          index = (original_index + j + j * j) & (new_num_slot - 1);

          // 이 경우는 발생하지 않아야 함 (new_num_slot이 충분히 큼)
          if (j >= new_num_slot) {
            throw std::runtime_error("Resize failed: new table is full");
          }
        }

        // 빈 슬롯에 데이터 복사 (복사 최적화)
        new_slots[index].key = std::move(slots_[i].key);
        new_slots[index].value = std::move(slots_[i].value);
        new_slots[index].is_occupied = true;
        ++new_activated;
      }
    }

    delete[] slots_;
    slots_ = new_slots;
    num_slot_ = new_num_slot;
    num_activated_slot_ = new_activated;
  }

  HashEntry<KeyType, ValueType>* slots_{nullptr};
  size_t num_slot_{0};
  size_t num_activated_slot_{0};
  double load_factor_{0.7};
  HashFunction hash_function_;
};

#endif  // OPEN_ADDRESSING_HASH_OPEN_ADDRESSING_HASH_MAP_H_
