#include "path_finder/Traits.hpp"

#include <doctest/doctest.h>

#include <cstddef>
#include <type_traits>
#include <vector>

namespace {

struct Arc {
  int from;
  int to;
};

struct MinimalTraits {
  using Node = int;
  using Arc = ::Arc;

  Node origin(const Arc& arc) const { return arc.from; }
  Node destination(const Arc& arc) const { return arc.to; }
  std::vector<Arc> outgoing(PathFinder::AgentIndex, const Node& node) const { return { { node, node + 1 } }; }
  PathFinder::FixedPoint duration(PathFinder::AgentIndex, const Arc&) const { return 10; }
  PathFinder::FixedPoint costs(PathFinder::AgentIndex, const Arc&) const { return 10; }
  bool canWait(PathFinder::AgentIndex, const Node&) const { return true; }
  PathFinder::FixedPoint waitingCost(PathFinder::AgentIndex) const { return 1; }
  PathFinder::Rect getBlockedArea(PathFinder::AgentIndex, const Arc&) const { return {}; }
  PathFinder::Rect getBlockedArea(PathFinder::AgentIndex, const Node&) const { return {}; }
};

struct Interval {
  int begin;
  int end;
};

struct CustomTraits : MinimalTraits {
  using Area = Interval;

  PathFinder::FixedPoint heuristic(PathFinder::AgentIndex, const Node& node, const Node& target) const {
    return node + target;
  }
  bool overlaps(const Area& area, const Area& otherArea) const { return area.begin < otherArea.end && otherArea.begin < area.end; }
  Area getBlockedArea(PathFinder::AgentIndex, const Arc& arc) const { return { arc.from, arc.to }; }
  Area getBlockedArea(PathFinder::AgentIndex, const Node& node) const { return { node, node }; }
};

} // namespace

static_assert( PathFinder::PathFinderTraits<MinimalTraits> );
static_assert( PathFinder::PathFinderTraits<CustomTraits> );
static_assert( !PathFinder::PathFinderTraits<Arc> );

static_assert( std::is_same_v<PathFinder::AreaOf<MinimalTraits>, PathFinder::Rect> );
static_assert( std::is_same_v<PathFinder::AreaOf<CustomTraits>, Interval> );

TEST_CASE("Traits defaults") {
  const MinimalTraits traits;

  CHECK( PathFinder::Detail::heuristic(traits, 0, 42, 2) == 0 );
  CHECK( PathFinder::Detail::overlaps(traits, PathFinder::Rect{ 0.0, 0.0, 1.0, 1.0 }, PathFinder::Rect{ 0.5, 0.5, 2.0, 2.0 }) );
}

TEST_CASE("Traits overrides") {
  const CustomTraits traits;

  CHECK( PathFinder::Detail::heuristic(traits, 0, 42, 2) == 44 );
  CHECK( PathFinder::Detail::overlaps(traits, Interval{ 0, 10 }, Interval{ 5, 15 }) );
  CHECK_FALSE( PathFinder::Detail::overlaps(traits, Interval{ 0, 10 }, Interval{ 10, 15 }) );
}
