#pragma once

#include "path_finder/Blocking.hpp"
#include "path_finder/SpatialIndex.hpp"
#include "path_finder/TimeInterval.hpp"
#include "path_finder/Traits.hpp"
#include "path_finder/Types.hpp"

#include <algorithm>
#include <cstddef>
#include <initializer_list>
#include <utility>
#include <vector>

namespace PathFinder {

/// Blockings of all agents. Each agent's blockings are stored per agent and registered in a spatial index
/// to find blockings of other agents overlapping a given area.
template<PathFinderTraits Traits, SpatialIndex<AreaOf<Traits>> Index>
class Blockings {
public:
  using Area = AreaOf<Traits>;
  using Node = NodeOf<Traits>;
  using Arc = ArcOf<Traits>;

  /// `traits` must outlive the blockings.
  Blockings(const Traits& traits, Index spatialIndex)
    : traits(traits)
    , spatialIndex(std::move(spatialIndex))
  {
  }

  void add(AgentIndex agentIndex, const Blocking<Area>& blocking) {
    blockingIds(agentIndex).push_back(store(agentIndex, blocking));
  }

  /// Removes all blockings of the agent equal to `blocking`.
  void remove(AgentIndex agentIndex, const Blocking<Area>& blocking) {
    std::erase_if(blockingIds(agentIndex), [&](std::size_t id) {
      if ( entries[id].blocking != blocking ) {
        return false;
      }
      release(id);
      return true;
    });
  }

  /// Trims the blockings of the agent to their parts outside `timeInterval`.
  void remove(AgentIndex agentIndex, const TimeInterval& timeInterval) {
    std::vector<Blocking<Area>> splitOff;
    std::erase_if(blockingIds(agentIndex), [&](std::size_t id) {
      Blocking<Area>& blocking = entries[id].blocking;
      if ( !blocking.times.overlaps(timeInterval) ) {
        return false;
      }
      const bool keepsBefore = blocking.times.begin < timeInterval.begin;
      const bool keepsAfter = timeInterval.end < blocking.times.end;
      if ( keepsBefore && keepsAfter ) {
        splitOff.push_back({ { timeInterval.end + 1, blocking.times.end }, blocking.area });
      }
      if ( keepsBefore ) {
        blocking.times.end = timeInterval.begin - 1;
        return false;
      }
      else if ( keepsAfter ) {
        blocking.times.begin = timeInterval.end + 1;
        return false;
      }
      release(id);
      return true;
    });
    for ( const Blocking<Area>& blocking : splitOff ) {
      add(agentIndex, blocking);
    }
  }

  /// Removes all blockings of the agent.
  void clear(AgentIndex agentIndex) {
    for ( std::size_t id : blockingIds(agentIndex) ) {
      release(id);
    }
    blockingIds(agentIndex).clear();
  }

  /// Blockings of the agent overlapping `timeInterval`, ordered by begin, equal begins in order of insertion.
  std::vector<Blocking<Area>> get(AgentIndex agentIndex, const TimeInterval& timeInterval) const {
    std::vector<Blocking<Area>> result;
    if ( agentIndex >= agentBlockingIds.size() ) {
      return result;
    }
    for ( std::size_t id : agentBlockingIds[agentIndex] ) {
      if ( entries[id].blocking.times.overlaps(timeInterval) ) {
        result.push_back(entries[id].blocking);
      }
    }
    std::ranges::stable_sort(result, {}, [](const Blocking<Area>& blocking) { return blocking.times.begin; });
    return result;
  }

  /// Maximal intervals within `timeInterval` in which the agent can use the arc, i.e. in which neither the area
  /// of the arc nor the areas of its origin and destination overlap blockings of other agents.
  std::vector<TimeInterval> availabilities(AgentIndex agentIndex, const TimeInterval& timeInterval, const Arc& arc) const {
    return freeTimes(agentIndex, timeInterval, {
      traits.getBlockedArea(agentIndex, arc),
      traits.getBlockedArea(agentIndex, traits.origin(arc)),
      traits.getBlockedArea(agentIndex, traits.destination(arc))
    });
  }

  /// Maximal intervals within `timeInterval` in which the area of the node does not overlap blockings of other agents.
  std::vector<TimeInterval> availabilities(AgentIndex agentIndex, const TimeInterval& timeInterval, const Node& node) const {
    return freeTimes(agentIndex, timeInterval, { traits.getBlockedArea(agentIndex, node) });
  }

private:
  /// An entry with agent index 0 is unused.
  struct Entry {
    AgentIndex agentIndex;
    Blocking<Area> blocking;
  };

  std::size_t store(AgentIndex agentIndex, const Blocking<Area>& blocking) {
    std::size_t id = entries.size();
    if ( unusedIds.empty() ) {
      entries.push_back({ agentIndex, blocking });
    }
    else {
      id = unusedIds.back();
      unusedIds.pop_back();
      entries[id] = { agentIndex, blocking };
    }
    spatialIndex.insert(id, blocking.area);
    return id;
  }

  void release(std::size_t id) {
    spatialIndex.erase(id, entries[id].blocking.area);
    entries[id].agentIndex = 0;
    unusedIds.push_back(id);
  }

  std::vector<std::size_t>& blockingIds(AgentIndex agentIndex) {
    if ( agentIndex >= agentBlockingIds.size() ) {
      agentBlockingIds.resize(agentIndex + 1);
    }
    return agentBlockingIds[agentIndex];
  }

  std::vector<TimeInterval> freeTimes(AgentIndex agentIndex, const TimeInterval& timeInterval, std::initializer_list<Area> areas) const {
    std::vector<TimeInterval> blockedTimes;
    for ( const Area& area : areas ) {
      spatialIndex.forEachCandidate(area, [&](std::size_t id) {
        const Entry& entry = entries[id];
        if ( entry.agentIndex != agentIndex &&
             entry.blocking.times.overlaps(timeInterval) &&
             Detail::overlaps(traits, entry.blocking.area, area) ) {
          blockedTimes.push_back(entry.blocking.times);
        }
      });
    }
    std::ranges::sort(blockedTimes, {}, &TimeInterval::begin);

    std::vector<TimeInterval> result;
    FixedPoint freeBegin = timeInterval.begin;
    for ( const TimeInterval& blocked : blockedTimes ) {
      if ( blocked.begin > freeBegin ) {
        result.push_back({ freeBegin, blocked.begin - 1 });
      }
      if ( blocked.end == TimeInterval::infinity ) {
        return result;
      }
      freeBegin = std::max(freeBegin, blocked.end + 1);
    }
    if ( freeBegin <= timeInterval.end ) {
      result.push_back({ freeBegin, timeInterval.end });
    }
    return result;
  }

  const Traits& traits;
  Index spatialIndex;
  std::vector<Entry> entries;
  std::vector<std::size_t> unusedIds;
  std::vector<std::vector<std::size_t>> agentBlockingIds;
};

} // namespace PathFinder
