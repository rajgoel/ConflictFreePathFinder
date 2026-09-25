#pragma once

#include <cstddef>

namespace PathFinder {

/// Requirements on a spatial index used to find blockings whose areas may overlap a given area.
/// `forEachCandidate` calls the callback with the id of every inserted area that may overlap `area`,
/// possibly more than once and possibly for areas that do not overlap. Exact checks are done by the caller.
template<typename Index, typename Area>
concept SpatialIndex =
  requires( Index& index, const Index& constIndex, std::size_t id, const Area& area, void (*callback)(std::size_t) ) {
    index.insert(id, area);
    index.erase(id, area);
    constIndex.forEachCandidate(area, callback);
  };

} // namespace PathFinder
