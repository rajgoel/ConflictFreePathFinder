#include "path_finder/defaults/GridIndex.hpp"

#include <doctest/doctest.h>

#include <algorithm>
#include <cstddef>
#include <vector>

using PathFinder::GridIndex;
using PathFinder::Rect;

namespace {

std::vector<std::size_t> candidates(const GridIndex<>& index, const Rect& area) {
  std::vector<std::size_t> ids;
  index.forEachCandidate(area, [&](std::size_t id) { ids.push_back(id); });
  std::ranges::sort(ids);
  const auto duplicates = std::ranges::unique(ids);
  ids.erase(duplicates.begin(), duplicates.end());
  return ids;
}

} // namespace

TEST_CASE("GridIndex finds areas in shared cells") {
  GridIndex<> index(1.0);
  index.insert(1, Rect{ 0.1, 0.1, 0.9, 0.9 });
  index.insert(2, Rect{ 5.1, 5.1, 5.9, 5.9 });
  index.insert(3, Rect{ 0.5, 0.5, 5.5, 0.9 });

  CHECK( candidates(index, Rect{ 0.2, 0.2, 0.3, 0.3 }) == std::vector<std::size_t>{ 1, 3 } );
  CHECK( candidates(index, Rect{ 5.2, 5.2, 5.3, 5.3 }) == std::vector<std::size_t>{ 2 } );
  CHECK( candidates(index, Rect{ 3.2, 0.2, 3.3, 0.3 }) == std::vector<std::size_t>{ 3 } );
  CHECK( candidates(index, Rect{ 3.2, 3.2, 3.3, 3.3 }).empty() );
}

TEST_CASE("GridIndex handles negative coordinates and erase") {
  GridIndex<> index(1.0);
  index.insert(1, Rect{ -1.9, -1.9, -1.1, -1.1 });

  CHECK( candidates(index, Rect{ -1.5, -1.5, -1.4, -1.4 }) == std::vector<std::size_t>{ 1 } );
  CHECK( candidates(index, Rect{ 0.5, 0.5, 0.6, 0.6 }).empty() );

  index.erase(1, Rect{ -1.9, -1.9, -1.1, -1.1 });
  CHECK( candidates(index, Rect{ -1.5, -1.5, -1.4, -1.4 }).empty() );
}
