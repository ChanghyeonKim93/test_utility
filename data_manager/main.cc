#include <iostream>
#include <vector>

#include "generic_key_value_data_manager.h"

struct Tile {
  int id{0};
  std::vector<uint8_t> data;
  ~Tile() {
    std::cerr << "delete tile id, data: " << id << ", " << data.size()
              << std::endl;
  }
};

struct Submap {
  int id{0};
  std::vector<uint8_t> data;
  ~Submap() {
    std::cerr << "delete submap id, data: " << id << ", " << data.size()
              << std::endl;
  }
};

int main() {
  try {
    GenericKeyValueDataManager data_manager;

    Tile tile;
    tile.id = 8;
    tile.data.resize(7);
    data_manager.AddData(tile.id, tile);

    Submap submap;
    submap.id = 5;
    submap.data.resize(3);
    data_manager.AddData(submap.id, submap);

    const auto existing_tile = data_manager.GetData<Tile>(tile.id);
    std::cerr << existing_tile.id << ", " << existing_tile.data.size()
              << std::endl;
    const auto existing_submap = data_manager.GetData<Submap>(submap.id);
    std::cerr << existing_submap.id << ", " << existing_submap.data.size()
              << std::endl;

    std::cout << "Start to delete data manager\n";
    data_manager.~GenericKeyValueDataManager();
    std::cout << "End of try\n";
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  std::cout << "wefoijwfe\n";
  return 0;
}