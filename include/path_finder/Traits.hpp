#pragma once

#include "path_finder/defaults/Rect.hpp"
#include "path_finder/Types.hpp"

#include <concepts>
#include <cstddef>
#include <functional>
#include <ranges>
#include <vector>

namespace PathFinder {

namespace Detail {

template<typename Traits>
struct AreaSelector {
  using Type = Rect;
};

template<typename Traits>
  requires requires { typename Traits::Area; }
struct AreaSelector<Traits> {
  using Type = typename Traits::Area;
};

} // namespace Detail

template<typename Traits>
using NodeOf = typename Traits::Node;

template<typename Traits>
using ArcOf = typename Traits::Arc;

/// Area type of the traits, defaults to `Rect` if the traits do not define `Area`.
template<typename Traits>
using AreaOf = typename Detail::AreaSelector<Traits>::Type;

/// Requirements on user-provided traits describing the graph and the agents moving on it.
/// Nodes must be equality comparable and hashable with `std::hash`.
///
/// Precondition for every agent and arc:
///   costs(agentIndex, arc) >= waitingCost(agentIndex) * duration(agentIndex, arc)
/// Otherwise driving in circles can be cheaper than waiting, and a search without path may not terminate.
///
/// Optional members:
///   FixedPoint heuristic(AgentIndex, const Node&, const Node& target) const  - defaults to 0
///     a lower bound of the costs from the node to the target, or TimeInterval::infinity where the target cannot be
///     reached from the node; it is asked for the node of each stage too, towards the node of the next
///   bool overlaps(const Area&, const Area&) const  - defaults to a free `overlaps` found by ADL
template<typename Traits>
concept PathFinderTraits =
  std::equality_comparable<NodeOf<Traits>> &&
  requires( const Traits& traits, AgentIndex agentIndex, const NodeOf<Traits>& node, const ArcOf<Traits>& arc ) {
    { std::hash<NodeOf<Traits>>{}(node) } -> std::convertible_to<std::size_t>;
    { traits.origin(arc) } -> std::convertible_to<NodeOf<Traits>>;
    { traits.destination(arc) } -> std::convertible_to<NodeOf<Traits>>;
    { traits.outgoing(agentIndex, node) } -> std::ranges::input_range;
    { traits.duration(agentIndex, arc) } -> std::convertible_to<FixedPoint>;
    { traits.costs(agentIndex, arc) } -> std::convertible_to<FixedPoint>;
    { traits.canWait(agentIndex, node) } -> std::convertible_to<bool>;
    { traits.waitingCost(agentIndex) } -> std::convertible_to<FixedPoint>;
    { traits.getBlockedArea(agentIndex, arc) } -> std::convertible_to<AreaOf<Traits>>;
    { traits.getBlockedArea(agentIndex, node) } -> std::convertible_to<AreaOf<Traits>>;
  };

namespace Detail {

template<PathFinderTraits Traits>
FixedPoint heuristic(const Traits& traits, AgentIndex agentIndex, const NodeOf<Traits>& node, const NodeOf<Traits>& target) {
  if constexpr ( requires { traits.heuristic(agentIndex, node, target); } ) {
    return traits.heuristic(agentIndex, node, target);
  }
  else {
    return 0;
  }
}

template<PathFinderTraits Traits>
bool overlaps(const Traits& traits, const AreaOf<Traits>& blockedArea, const AreaOf<Traits>& otherBlockedArea) {
  if constexpr ( requires { traits.overlaps(blockedArea, otherBlockedArea); } ) {
    return traits.overlaps(blockedArea, otherBlockedArea);
  }
  else {
    return overlaps(blockedArea, otherBlockedArea);
  }
}

} // namespace Detail

} // namespace PathFinder
