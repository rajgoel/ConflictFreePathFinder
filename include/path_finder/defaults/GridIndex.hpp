#pragma once

#include "path_finder/defaults/Rect.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace PathFinder {

/// Default spatial index: a uniform grid of square cells. Each area is registered in all cells covered by
/// its bounding box, obtained by a free `boundingBox(area)` returning a `Rect`, found by ADL.
template<typename Area = Rect>
class GridIndex {
public:
  explicit GridIndex(double cellSize)
    : cellSize(cellSize)
  {
  }

  void insert(std::size_t id, const Area& area) {
    forEachCell(area, [&](std::int64_t cellKey) {
      cells[cellKey].push_back(id);
    });
  }

  void erase(std::size_t id, const Area& area) {
    forEachCell(area, [&](std::int64_t cellKey) {
      auto cell = cells.find(cellKey);
      if ( cell == cells.end() ) {
        return;
      }
      std::erase(cell->second, id);
      if ( cell->second.empty() ) {
        cells.erase(cell);
      }
    });
  }

  template<typename Callback>
  void forEachCandidate(const Area& area, Callback&& callback) const {
    forEachCell(area, [&](std::int64_t cellKey) {
      auto cell = cells.find(cellKey);
      if ( cell == cells.end() ) {
        return;
      }
      for ( std::size_t id : cell->second ) {
        callback(id);
      }
    });
  }

private:
  template<typename Visitor>
  void forEachCell(const Area& area, Visitor&& visitor) const {
    const Rect& box = boundingBox(area);
    const std::int32_t minCellX = cellCoordinate(box.minX);
    const std::int32_t maxCellX = cellCoordinate(box.maxX);
    const std::int32_t minCellY = cellCoordinate(box.minY);
    const std::int32_t maxCellY = cellCoordinate(box.maxY);
    for ( std::int32_t cellX = minCellX; cellX <= maxCellX; ++cellX ) {
      for ( std::int32_t cellY = minCellY; cellY <= maxCellY; ++cellY ) {
        visitor(cellKey(cellX, cellY));
      }
    }
  }

  std::int32_t cellCoordinate(double coordinate) const {
    return static_cast<std::int32_t>(std::floor(coordinate / cellSize));
  }

  static std::int64_t cellKey(std::int32_t cellX, std::int32_t cellY) {
    return static_cast<std::int64_t>(static_cast<std::uint64_t>(static_cast<std::uint32_t>(cellX)) << 32 |
                                     static_cast<std::uint32_t>(cellY));
  }

  double cellSize;
  std::unordered_map<std::int64_t, std::vector<std::size_t>> cells;
};

} // namespace PathFinder
