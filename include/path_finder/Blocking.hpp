#pragma once

#include "path_finder/TimeInterval.hpp"

namespace PathFinder {

/// Area occupied by an agent during a time interval.
template<typename Area>
struct Blocking {
  TimeInterval times;
  Area area;

  friend constexpr bool operator==(const Blocking&, const Blocking&) = default;
};

} // namespace PathFinder
