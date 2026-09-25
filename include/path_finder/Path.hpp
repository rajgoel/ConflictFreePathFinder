#pragma once

#include "path_finder/TimeInterval.hpp"
#include "path_finder/Traits.hpp"

#include <vector>

namespace PathFinder {

/// Stay of an agent at a node, from arrival (`times.begin`) to departure (`times.end`).
/// The last waypoint of a path ends at `TimeInterval::infinity`.
template<PathFinderTraits Traits>
struct Waypoint {
  TimeInterval times;
  NodeOf<Traits> node;
};

/// Timed path of an agent. `arcs[i]` leads from `waypoints[i].node` to `waypoints[i + 1].node`.
template<PathFinderTraits Traits>
struct Path {
  std::vector<Waypoint<Traits>> waypoints;
  std::vector<ArcOf<Traits>> arcs;
};

} // namespace PathFinder
