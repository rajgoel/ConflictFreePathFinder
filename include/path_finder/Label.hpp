#pragma once

#include "path_finder/TimeInterval.hpp"
#include "path_finder/Traits.hpp"
#include "path_finder/Types.hpp"

#include <cstddef>
#include <memory>
#include <optional>

namespace PathFinder {

/// Search state of an agent at a node. `timeWindow` holds the times the agent can be at the node,
/// `costs` the costs (without heuristic) of being there at `timeWindow.begin`. Being there later costs
/// `waitingCost` per time unit. `stage` is the number of completed stages. `arc` is the arc leading to
/// `node` from the predecessor, empty for start labels.
template<PathFinderTraits Traits>
struct Label {
  NodeOf<Traits> node;
  TimeInterval timeWindow;
  FixedPoint costs;
  std::size_t stage;
  std::shared_ptr<const Label> predecessor;
  std::optional<ArcOf<Traits>> arc;
  bool dominated = false;

  /// Both labels must belong to the same agent and node. A label with a higher stage can complete every
  /// remaining path of the other label. A label with an earlier begin has to wait to reach the begin of
  /// the other label, so the waiting costs are added before comparing costs.
  bool dominates(const Label& other, FixedPoint waitingCost) const {
    return timeWindow.begin <= other.timeWindow.begin &&
           timeWindow.end >= other.timeWindow.end &&
           stage >= other.stage &&
           costs + waitingCost * ( other.timeWindow.begin - timeWindow.begin ) <= other.costs;
  }
};

} // namespace PathFinder
