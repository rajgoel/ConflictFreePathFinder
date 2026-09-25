#pragma once

#include "path_finder/Blocking.hpp"
#include "path_finder/Blockings.hpp"
#include "path_finder/Path.hpp"
#include "path_finder/Search.hpp"
#include "path_finder/SpatialIndex.hpp"
#include "path_finder/TimeInterval.hpp"
#include "path_finder/Traits.hpp"
#include "path_finder/Types.hpp"
#include "path_finder/defaults/GridIndex.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace PathFinder {

/// Collaborative path finder. Paths are planned one agent at a time: `findPath` searches a path avoiding all
/// committed blockings of other agents, `updatePath` commits it. An agent with a path rests at the last node
/// of its path until the end of time.
template<PathFinderTraits Traits, SpatialIndex<AreaOf<Traits>> Index = GridIndex<AreaOf<Traits>>>
class PathFinder {
public:
  using Node = NodeOf<Traits>;
  using Arc = ArcOf<Traits>;
  using Stages = std::vector<Node>;

  PathFinder(Traits traits, Index spatialIndex)
    : traits(std::move(traits))
    , blockings(this->traits, std::move(spatialIndex))
    , paths(1)
  {
  }

  /// The blockings refer to the traits of this path finder, so it can neither be copied nor moved.
  PathFinder(const PathFinder&) = delete;
  PathFinder& operator=(const PathFinder&) = delete;

  /// Adds an agent resting at `node` from `time` on. Returns 0 if the node is not free from `time` until infinity.
  AgentIndex addAgent(FixedPoint time, const Node& node) {
    assert( time >= currentTime );
    const AgentIndex agentIndex = static_cast<AgentIndex>(paths.size());
    const TimeInterval forever = { time, TimeInterval::infinity };
    const std::vector<TimeInterval> freeTimes = blockings.availabilities(agentIndex, forever, node);
    if ( freeTimes.size() != 1 || freeTimes.front() != forever ) {
      return 0;
    }
    paths.push_back({ { { forever, node } }, {} });
    addBlockings(agentIndex, paths.back());
    return agentIndex;
  }

  /// Removes the path of the agent, which then no longer blocks anything until its path is updated again.
  void clearPath(AgentIndex agentIndex) {
    paths[agentIndex] = {};
    blockings.clear(agentIndex);
  }

  /// Searches the cheapest path starting at `time` and visiting the node of each stage in order. The path starts where
  /// the current path of the agent is at `time`, or at the arrival of the arc the agent is on at `time`. An agent without
  /// path starts at the node of the first stage at `time`. The caller ensures that the node of the final stage allows
  /// waiting. Returns an empty path if there is none. Nothing is committed.
  Path<Traits> findPath(AgentIndex agentIndex, FixedPoint time, const Stages& stages) const {
    assert( time >= currentTime );
    if ( stages.empty() ) {
      return {};
    }
    Detail::Search<Traits, Index> search(traits, blockings, agentIndex, stages, time);
    const Path<Traits>& currentPath = paths[agentIndex];
    if ( currentPath.waypoints.empty() ) {
      search.addStart(stages.front(), time);
    }
    else {
      const Waypoint<Traits>& start = currentPath.waypoints[startIndex(currentPath, time)];
      search.addStart(start.node, std::max(time, start.times.begin));
    }
    return search.run();
  }

  Path<Traits> findPath(AgentIndex agentIndex, FixedPoint time, const Node& node) const {
    return findPath(agentIndex, time, Stages{ node });
  }

  /// Replaces the path of the agent from `time` on by `path`, which must have been found by `findPath` for the same
  /// agent and time and still be free of conflicts. The path is not checked.
  void updatePath(AgentIndex agentIndex, FixedPoint time, const Path<Traits>& path) {
    assert( time >= currentTime );
    assert( !path.waypoints.empty() );
    Path<Traits>& currentPath = paths[agentIndex];
    if ( currentPath.waypoints.empty() ) {
      currentPath = path;
    }
    else {
      const std::size_t start = startIndex(currentPath, time);
      assert( currentPath.waypoints[start].node == path.waypoints.front().node );
      const FixedPoint arrival = currentPath.waypoints[start].times.begin;
      currentPath.waypoints.erase(currentPath.waypoints.begin() + static_cast<std::ptrdiff_t>(start), currentPath.waypoints.end());
      currentPath.arcs.erase(currentPath.arcs.begin() + static_cast<std::ptrdiff_t>(start), currentPath.arcs.end());
      currentPath.waypoints.insert(currentPath.waypoints.end(), path.waypoints.begin(), path.waypoints.end());
      currentPath.arcs.insert(currentPath.arcs.end(), path.arcs.begin(), path.arcs.end());
      currentPath.waypoints[start].times.begin = arrival;
    }
    blockings.clear(agentIndex);
    addBlockings(agentIndex, currentPath);
  }

  const Path<Traits>& getPath(AgentIndex agentIndex) const {
    return paths[agentIndex];
  }

  /// The current time, which starts at 0 and only moves forward.
  FixedPoint now() const {
    return currentTime;
  }

  /// Moves the current time forward to `time` and forgets what has finished before it: the waypoints and arcs
  /// agents have completed, and the parts of blockings before `time`. Nothing may be searched, committed or
  /// added before the current time afterwards.
  void advance(FixedPoint time) {
    assert( time >= currentTime );
    currentTime = time;
    for ( AgentIndex agentIndex = 1; agentIndex < paths.size(); ++agentIndex ) {
      forgetFinished(paths[agentIndex], time);
      blockings.remove(agentIndex, TimeInterval{ std::numeric_limits<FixedPoint>::min(), time - 1 });
    }
  }

  const Blockings<Traits, Index>& getBlockings() const {
    return blockings;
  }

private:
  /// Index of the first waypoint the agent has not left before `time`.
  static std::size_t startIndex(const Path<Traits>& path, FixedPoint time) {
    const auto start = std::ranges::find_if(path.waypoints, [time](const Waypoint<Traits>& waypoint) {
      return waypoint.times.end >= time;
    });
    return static_cast<std::size_t>(start - path.waypoints.begin());
  }

  /// Removes the waypoints the agent has left and the arcs it has completed before `time`. An arc it arrives from
  /// at `time` is still occupied at that instant and kept, together with the waypoint it leaves.
  static void forgetFinished(Path<Traits>& path, FixedPoint time) {
    std::size_t finished = 0;
    while ( finished + 1 < path.waypoints.size() && path.waypoints[finished + 1].times.begin < time ) {
      ++finished;
    }
    const auto count = static_cast<std::ptrdiff_t>(finished);
    path.waypoints.erase(path.waypoints.begin(), path.waypoints.begin() + count);
    path.arcs.erase(path.arcs.begin(), path.arcs.begin() + count);
  }

  /// A node is blocked from the departure towards it until the arrival at the next node, an arc during its traversal.
  /// Nothing is blocked before the current time.
  void addBlockings(AgentIndex agentIndex, const Path<Traits>& path) {
    const std::vector<Waypoint<Traits>>& waypoints = path.waypoints;
    for ( std::size_t index = 0; index < waypoints.size(); ++index ) {
      const FixedPoint begin = index == 0 ? waypoints[index].times.begin : waypoints[index - 1].times.end;
      const FixedPoint end = index + 1 == waypoints.size() ? waypoints[index].times.end : waypoints[index + 1].times.begin;
      addBlocking(agentIndex, { begin, end }, traits.getBlockedArea(agentIndex, waypoints[index].node));
    }
    for ( std::size_t index = 0; index < path.arcs.size(); ++index ) {
      const TimeInterval traversal = { waypoints[index].times.end, waypoints[index + 1].times.begin };
      addBlocking(agentIndex, traversal, traits.getBlockedArea(agentIndex, path.arcs[index]));
    }
  }

  void addBlocking(AgentIndex agentIndex, const TimeInterval& times, const AreaOf<Traits>& area) {
    const TimeInterval remaining = times.intersection({ currentTime, TimeInterval::infinity });
    if ( !remaining.empty() ) {
      blockings.add(agentIndex, { remaining, area });
    }
  }

  Traits traits;
  FixedPoint currentTime = 0;
  Blockings<Traits, Index> blockings;
  /// Paths indexed by agent index; index 0 is unused.
  std::vector<Path<Traits>> paths;
};

} // namespace PathFinder
