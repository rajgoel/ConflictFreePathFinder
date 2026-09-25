#include "path_finder/Label.hpp"

#include <doctest/doctest.h>

#include <vector>

namespace {

struct Arc {
  int from;
  int to;
};

struct LabelTraits {
  using Node = int;
  using Arc = ::Arc;

  Node origin(const Arc& arc) const { return arc.from; }
  Node destination(const Arc& arc) const { return arc.to; }
  std::vector<Arc> outgoing(PathFinder::AgentIndex, const Node&) const { return {}; }
  PathFinder::FixedPoint duration(PathFinder::AgentIndex, const Arc&) const { return 10; }
  PathFinder::FixedPoint costs(PathFinder::AgentIndex, const Arc&) const { return 10; }
  bool canWait(PathFinder::AgentIndex, const Node&) const { return true; }
  PathFinder::FixedPoint waitingCost(PathFinder::AgentIndex) const { return 1; }
  PathFinder::Rect getBlockedArea(PathFinder::AgentIndex, const Arc&) const { return {}; }
  PathFinder::Rect getBlockedArea(PathFinder::AgentIndex, const Node&) const { return {}; }
};

using Label = PathFinder::Label<LabelTraits>;

Label makeLabel(PathFinder::TimeInterval timeWindow, PathFinder::FixedPoint costs, std::size_t stage = 0) {
  return Label{ 0, timeWindow, costs, stage, nullptr, std::nullopt };
}

} // namespace

TEST_CASE("Label dominates labels with contained window, lower stage and higher costs") {
  const Label label = makeLabel({ 0, 100 }, 10, 1);

  CHECK( label.dominates(makeLabel({ 0, 100 }, 10, 1), 0) );
  CHECK( label.dominates(makeLabel({ 10, 50 }, 30, 1), 1) );
  CHECK( label.dominates(makeLabel({ 0, 100 }, 10, 0), 0) );
  CHECK_FALSE( label.dominates(makeLabel({ 0, 100 }, 9, 1), 0) );
  CHECK_FALSE( label.dominates(makeLabel({ 0, 101 }, 10, 1), 0) );
  CHECK_FALSE( label.dominates(makeLabel({ 0, 100 }, 10, 2), 0) );
}

TEST_CASE("Label dominance accounts for waiting to reach the later begin") {
  const Label early = makeLabel({ 0, 100 }, 10);
  const Label late = makeLabel({ 20, 100 }, 25);

  CHECK( early.dominates(late, 0) );
  CHECK_FALSE( early.dominates(late, 1) );
  CHECK( early.dominates(makeLabel({ 20, 100 }, 30), 1) );
}
