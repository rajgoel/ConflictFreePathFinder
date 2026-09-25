#pragma once

#include "path_finder/Blockings.hpp"
#include "path_finder/Label.hpp"
#include "path_finder/Path.hpp"
#include "path_finder/TimeInterval.hpp"
#include "path_finder/Traits.hpp"
#include "path_finder/Types.hpp"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace PathFinder::Detail {

/// Shortest path search with time windows for a single agent through a sequence of stages, each stage being
/// a node to visit. Labels are expanded in order of costs plus a lower bound of the costs still to come: the
/// heuristic to the node of the current stage, and from the node of each stage to the next. The search ends with
/// the first label completing all stages; the final stage is only completed if its node is free until infinity.
/// A label from which the heuristic finds the node of its stage out of reach is dropped.
///
/// The times an arc or a node is free for the agent are worked out once per search, from the start of the search
/// on, and every label reaching it looks up its own window in them. An arc is known by the node it leaves and its
/// position among the arcs leaving that node, which stays the same throughout a search.
template<PathFinderTraits Traits, typename Index>
class Search {
public:
  using Node = NodeOf<Traits>;
  using Arc = ArcOf<Traits>;
  using Stages = std::vector<Node>;

  /// All arguments must outlive the search, which considers no time before `start`.
  Search(const Traits& traits, const Blockings<Traits, Index>& blockings, AgentIndex agentIndex, const Stages& stages,
         FixedPoint start)
    : traits(traits)
    , blockings(blockings)
    , agentIndex(agentIndex)
    , stages(stages)
    , start(start)
    , waitingCost(traits.waitingCost(agentIndex))
    , following(stages.size(), 0)
  {
    for ( std::size_t stage = stages.size(); stage > 1; --stage ) {
      following[stage - 2] = sum(heuristic(traits, agentIndex, stages[stage - 2], stages[stage - 1]), following[stage - 1]);
    }
  }

  /// Adds a start label for the agent being at `node` at time `begin`, which must not precede the start of the search.
  /// Ignored if the node is not free at `begin`.
  void addStart(const Node& node, FixedPoint begin) {
    const auto free = freeAt(freeTimes(node), begin);
    if ( !free ) {
      return;
    }
    const TimeInterval timeWindow = { begin, traits.canWait(agentIndex, node) ? free->end : begin };
    addLabel({ node, timeWindow, 0, advancedStage(0, node, timeWindow), nullptr, std::nullopt });
  }

  /// Returns the cheapest path completing all stages, or an empty path if there is none.
  Path<Traits> run() {
    while ( !queue.empty() ) {
      const std::shared_ptr<const Label<Traits>> label = queue.top().label;
      queue.pop();
      if ( label->dominated ) {
        continue;
      }
      if ( label->stage == stages.size() ) {
        return path(*label);
      }
      expand(label);
    }
    return {};
  }

private:
  struct QueueEntry {
    FixedPoint priority;
    std::shared_ptr<const Label<Traits>> label;

    bool operator>(const QueueEntry& other) const {
      return priority > other.priority;
    }
  };

  void expand(const std::shared_ptr<const Label<Traits>>& label) {
    std::size_t position = 0;
    for ( const Arc& arc : traits.outgoing(agentIndex, label->node) ) {
      const FixedPoint duration = traits.duration(agentIndex, arc);
      const TimeInterval usage = { label->timeWindow.begin, label->timeWindow.shifted(duration).end };
      const std::vector<TimeInterval>& freeForArc = freeTimes(label->node, position++, arc);
      auto free = std::ranges::lower_bound(freeForArc, usage.begin, {}, &TimeInterval::end);
      for ( ; free != freeForArc.end() && free->begin <= usage.end; ++free ) {
        const TimeInterval usable = free->intersection(usage);
        const TimeInterval departures = label->timeWindow.intersection({ usable.begin, usable.shifted(-duration).end });
        if ( departures.empty() ) {
          continue;
        }
        const Node destination = traits.destination(arc);
        const TimeInterval timeWindow = timeWindowAt(destination, departures.shifted(duration));
        const FixedPoint waitingTime = departures.begin - label->timeWindow.begin;
        addLabel({
          destination,
          timeWindow,
          label->costs + traits.costs(agentIndex, arc) + waitingCost * waitingTime,
          advancedStage(label->stage, destination, timeWindow),
          label,
          arc
        });
      }
    }
  }

  /// Times the agent can be at `node` when arriving within `arrivals`, which must be free for the agent.
  /// If waiting is allowed, the agent can stay until the node stops being free.
  TimeInterval timeWindowAt(const Node& node, const TimeInterval& arrivals) {
    if ( !traits.canWait(agentIndex, node) ) {
      return arrivals;
    }
    return { arrivals.begin, freeAt(freeTimes(node), arrivals.end)->end };
  }

  /// The times the node is free for the agent from the start of the search on.
  const std::vector<TimeInterval>& freeTimes(const Node& node) {
    auto [known, added] = nodeFreeTimes.try_emplace(node);
    if ( added ) {
      known->second = blockings.availabilities(agentIndex, { start, TimeInterval::infinity }, node);
    }
    return known->second;
  }

  /// The times the arc at `position` among those leaving `origin` is free for the agent from the start of the
  /// search on.
  const std::vector<TimeInterval>& freeTimes(const Node& origin, std::size_t position, const Arc& arc) {
    std::vector<std::optional<std::vector<TimeInterval>>>& leaving = arcFreeTimes[origin];
    if ( position >= leaving.size() ) {
      leaving.resize(position + 1);
    }
    if ( !leaving[position] ) {
      leaving[position] = blockings.availabilities(agentIndex, { start, TimeInterval::infinity }, arc);
    }
    return *leaving[position];
  }

  /// The interval of `freeTimes` holding `time`, if any.
  static std::optional<TimeInterval> freeAt(const std::vector<TimeInterval>& freeTimes, FixedPoint time) {
    const auto free = std::ranges::lower_bound(freeTimes, time, {}, &TimeInterval::end);
    if ( free == freeTimes.end() || !free->contains(time) ) {
      return std::nullopt;
    }
    return *free;
  }

  /// Stage after being at `node` within `timeWindow`. The final stage is only completed if the agent can stay forever.
  std::size_t advancedStage(std::size_t stage, const Node& node, const TimeInterval& timeWindow) const {
    if ( stage == stages.size() || stages[stage] != node ) {
      return stage;
    }
    const bool isFinalStage = stage + 1 == stages.size();
    if ( isFinalStage && timeWindow.end != TimeInterval::infinity ) {
      return stage;
    }
    return stage + 1;
  }

  void addLabel(Label<Traits> candidate) {
    const FixedPoint estimate = priority(candidate);
    if ( estimate == TimeInterval::infinity ) {
      return;
    }
    std::vector<std::shared_ptr<Label<Traits>>>& labelsAtNode = labels[candidate.node];
    for ( const std::shared_ptr<Label<Traits>>& label : labelsAtNode ) {
      if ( label->dominates(candidate, waitingCost) ) {
        return;
      }
    }
    for ( const std::shared_ptr<Label<Traits>>& label : labelsAtNode ) {
      label->dominated = candidate.dominates(*label, waitingCost);
    }
    std::erase_if(labelsAtNode, [](const std::shared_ptr<Label<Traits>>& label) { return label->dominated; });

    auto label = std::make_shared<Label<Traits>>(std::move(candidate));
    labelsAtNode.push_back(label);
    queue.push({ estimate, label });
  }

  /// Costs plus a lower bound of the costs still to come, infinity where the node of the current stage is out of reach.
  FixedPoint priority(const Label<Traits>& label) const {
    if ( label.stage == stages.size() ) {
      return label.costs;
    }
    return sum(label.costs, sum(heuristic(traits, agentIndex, label.node, stages[label.stage]), following[label.stage]));
  }

  /// The sum of two costs, infinity where either is.
  static FixedPoint sum(FixedPoint costs, FixedPoint otherCosts) {
    if ( costs == TimeInterval::infinity || otherCosts == TimeInterval::infinity ) {
      return TimeInterval::infinity;
    }
    return costs + otherCosts;
  }


  /// Builds the path ending with `finalLabel`, arriving as early as possible at the final node. Going backwards,
  /// each departure follows from the next arrival. The agent arrives as early as possible where it can wait,
  /// and immediately before departure where it cannot.
  Path<Traits> path(const Label<Traits>& finalLabel) const {
    Path<Traits> result;
    FixedPoint departure = TimeInterval::infinity;
    for ( const Label<Traits>* label = &finalLabel; label; label = label->predecessor.get() ) {
      const bool arrivesEarliest = departure == TimeInterval::infinity || traits.canWait(agentIndex, label->node);
      const FixedPoint arrival = arrivesEarliest ? label->timeWindow.begin : departure;
      result.waypoints.push_back({ { arrival, departure }, label->node });
      if ( label->arc ) {
        result.arcs.push_back(*label->arc);
        departure = arrival - traits.duration(agentIndex, *label->arc);
      }
    }
    std::ranges::reverse(result.waypoints);
    std::ranges::reverse(result.arcs);
    return result;
  }

  const Traits& traits;
  const Blockings<Traits, Index>& blockings;
  AgentIndex agentIndex;
  const Stages& stages;
  FixedPoint start;
  FixedPoint waitingCost;
  std::vector<FixedPoint> following; // by stage, a lower bound of the costs from its node through all later stages
  std::unordered_map<Node, std::vector<TimeInterval>> nodeFreeTimes;
  std::unordered_map<Node, std::vector<std::optional<std::vector<TimeInterval>>>> arcFreeTimes;
  std::unordered_map<Node, std::vector<std::shared_ptr<Label<Traits>>>> labels;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> queue;
};

} // namespace PathFinder::Detail
