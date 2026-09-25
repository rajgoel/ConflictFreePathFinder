#pragma once

#include "path_finder/Types.hpp"

#include <algorithm>
#include <limits>

namespace PathFinder {

/// Closed time interval [begin, end]. A single instant [t, t] is a valid, non-empty interval, e.g. for
/// nodes that do not allow waiting. Closed intervals sharing an endpoint overlap, so an area freed at t
/// can only be taken again at t + 1. An end of `infinity` denotes an unbounded interval.
struct TimeInterval {
  static constexpr FixedPoint infinity = std::numeric_limits<FixedPoint>::max();

  FixedPoint begin = 0;
  FixedPoint end = infinity;

  constexpr bool empty() const {
    return begin > end;
  }

  constexpr bool contains(FixedPoint time) const {
    return begin <= time && time <= end;
  }

  constexpr bool overlaps(const TimeInterval& other) const {
    return begin <= other.end && other.begin <= end;
  }

  constexpr TimeInterval intersection(const TimeInterval& other) const {
    return { std::max(begin, other.begin), std::min(end, other.end) };
  }

  /// Moves the interval by `offset`, keeping an unbounded end unbounded.
  constexpr TimeInterval shifted(FixedPoint offset) const {
    return { begin + offset, end == infinity ? infinity : end + offset };
  }

  friend constexpr bool operator==(const TimeInterval&, const TimeInterval&) = default;
};

} // namespace PathFinder
