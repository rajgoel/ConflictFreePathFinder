#include "GridWorld.hpp"

#include <doctest/doctest.h>

#include <vector>

using PathFinder::TimeInterval;
using GridWorld::Cell;
using Stages = GridWorld::PathFinder::Stages;

namespace {

std::vector<Cell> cells(const PathFinder::Path<GridWorld::Traits>& path) {
  std::vector<Cell> result;
  for ( const auto& waypoint : path.waypoints ) {
    result.push_back(waypoint.node);
  }
  return result;
}

std::vector<TimeInterval> times(const PathFinder::Path<GridWorld::Traits>& path) {
  std::vector<TimeInterval> result;
  for ( const auto& waypoint : path.waypoints ) {
    result.push_back(waypoint.times);
  }
  return result;
}

constexpr PathFinder::FixedPoint infinity = TimeInterval::infinity;

} // namespace

TEST_CASE("PathFinder finds a straight path") {
  auto pathFinder = GridWorld::makePathFinder({ 4, 1 });
  const PathFinder::AgentIndex agent = pathFinder.addAgent(0, { 0, 0 });
  REQUIRE( agent == 1 );

  const auto path = pathFinder.findPath(agent, 5, Cell{ 3, 0 });

  CHECK( cells(path) == std::vector<Cell>{ { 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 } } );
  CHECK( times(path) == std::vector<TimeInterval>{ { 5, 5 }, { 15, 15 }, { 25, 25 }, { 35, infinity } } );
  CHECK( path.arcs.size() == 3 );
}

TEST_CASE("PathFinder addAgent fails on a node that is not free forever") {
  auto pathFinder = GridWorld::makePathFinder({ 4, 1 });
  REQUIRE( pathFinder.addAgent(0, { 1, 0 }) == 1 );

  CHECK( pathFinder.addAgent(0, { 1, 0 }) == 0 );
  CHECK( pathFinder.addAgent(0, { 2, 0 }) == 2 );
}

TEST_CASE("PathFinder returns an empty path if the target is blocked") {
  auto pathFinder = GridWorld::makePathFinder({ 4, 1 });
  const PathFinder::AgentIndex agent = pathFinder.addAgent(0, { 0, 0 });
  pathFinder.addAgent(0, { 2, 0 });

  CHECK( pathFinder.findPath(agent, 0, Cell{ 3, 0 }).waypoints.empty() );
  CHECK( pathFinder.findPath(agent, 0, Cell{ 1, 0 }).waypoints.size() == 2 );
}

TEST_CASE("PathFinder avoids a committed path by waiting or by a detour depending on waiting costs") {
  // Agent 1 rests at (1,1) until 500 and then moves to (1,2). Agent 2 wants to get from (0,1) to (2,1).
  for ( PathFinder::FixedPoint waitingCost : { 0, 1 } ) {
    auto pathFinder = GridWorld::makePathFinder({ 3, 3, waitingCost });
    const PathFinder::AgentIndex first = pathFinder.addAgent(0, { 1, 1 });
    const PathFinder::AgentIndex second = pathFinder.addAgent(0, { 0, 1 });
    pathFinder.updatePath(first, 500, pathFinder.findPath(first, 500, Cell{ 1, 2 }));

    const auto secondPath = pathFinder.findPath(second, 0, Cell{ 2, 1 });
    if ( waitingCost == 0 ) {
      CHECK( cells(secondPath) == std::vector<Cell>{ { 0, 1 }, { 1, 1 }, { 2, 1 } } );
      CHECK( times(secondPath) == std::vector<TimeInterval>{ { 0, 511 }, { 521, 521 }, { 531, infinity } } );
    }
    else {
      // The detours through the lower and the upper row are equally cheap.
      CHECK( secondPath.arcs.size() == 4 );
      CHECK( times(secondPath).back() == TimeInterval{ 40, infinity } );
    }
  }
}

TEST_CASE("PathFinder waits before a node that does not allow waiting") {
  // Agent 1 moves from (2,0) through (2,1) to (2,2). Agent 2 moves from (0,1) through (1,1) to (2,1).
  for ( bool canWaitInBetween : { true, false } ) {
    auto pathFinder = GridWorld::makePathFinder({ 3, 3, 0, canWaitInBetween ? std::vector<Cell>{} : std::vector<Cell>{ { 1, 1 } } });
    const PathFinder::AgentIndex first = pathFinder.addAgent(0, { 2, 0 });
    const PathFinder::AgentIndex second = pathFinder.addAgent(0, { 0, 1 });
    pathFinder.updatePath(first, 0, pathFinder.findPath(first, 0, Cell{ 2, 2 }));

    const auto secondPath = pathFinder.findPath(second, 0, Cell{ 2, 1 });

    CHECK( cells(secondPath) == std::vector<Cell>{ { 0, 1 }, { 1, 1 }, { 2, 1 } } );
    if ( canWaitInBetween ) {
      CHECK( times(secondPath) == std::vector<TimeInterval>{ { 0, 0 }, { 10, 21 }, { 31, infinity } } );
    }
    else {
      CHECK( times(secondPath) == std::vector<TimeInterval>{ { 0, 11 }, { 21, 21 }, { 31, infinity } } );
    }
  }
}

TEST_CASE("PathFinder visits stages in order") {
  auto pathFinder = GridWorld::makePathFinder({ 3, 1 });
  const PathFinder::AgentIndex agent = pathFinder.addAgent(0, { 0, 0 });

  const auto path = pathFinder.findPath(agent, 0, Stages{ { 2, 0 }, { 0, 0 } });

  CHECK( cells(path) == std::vector<Cell>{ { 0, 0 }, { 1, 0 }, { 2, 0 }, { 1, 0 }, { 0, 0 } } );
}

TEST_CASE("PathFinder only ends at a final node that is free forever") {
  auto pathFinder = GridWorld::makePathFinder({ 5, 2 });
  const PathFinder::AgentIndex first = pathFinder.addAgent(0, { 4, 0 });
  const PathFinder::AgentIndex second = pathFinder.addAgent(0, { 0, 1 });
  // Agent 1 passes (1,0) between 1020 and 1040 on its way to (0,0), where it stays.
  pathFinder.updatePath(first, 1000, pathFinder.findPath(first, 1000, Cell{ 0, 0 }));

  const auto late = pathFinder.findPath(second, 0, Cell{ 1, 0 });
  REQUIRE( !late.waypoints.empty() );
  CHECK( late.waypoints.back().times == TimeInterval{ 1051, infinity } );
}

TEST_CASE("PathFinder keeps the path until the given time") {
  auto pathFinder = GridWorld::makePathFinder({ 4, 1 });
  const PathFinder::AgentIndex agent = pathFinder.addAgent(0, { 0, 0 });
  pathFinder.updatePath(agent, 0, pathFinder.findPath(agent, 0, Cell{ 3, 0 }));

  // At time 15 the agent is on the arc to (2,0), so the new path starts there at time 20.
  const auto path = pathFinder.findPath(agent, 15, Cell{ 0, 0 });
  REQUIRE( cells(path) == std::vector<Cell>{ { 2, 0 }, { 1, 0 }, { 0, 0 } } );
  CHECK( times(path).front() == TimeInterval{ 20, 20 } );

  pathFinder.updatePath(agent, 15, path);
  CHECK( cells(pathFinder.getPath(agent)) == std::vector<Cell>{ { 0, 0 }, { 1, 0 }, { 2, 0 }, { 1, 0 }, { 0, 0 } } );
  CHECK( times(pathFinder.getPath(agent)) == std::vector<TimeInterval>{ { 0, 0 }, { 10, 10 }, { 20, 20 }, { 30, 30 }, { 40, infinity } } );
}

TEST_CASE("PathFinder lets a cleared agent reappear at the node of the first stage") {
  auto pathFinder = GridWorld::makePathFinder({ 4, 1 });
  const PathFinder::AgentIndex agent = pathFinder.addAgent(0, { 0, 0 });
  const PathFinder::AgentIndex other = pathFinder.addAgent(0, { 3, 0 });

  pathFinder.clearPath(agent);
  CHECK( pathFinder.getPath(agent).waypoints.empty() );
  CHECK( pathFinder.findPath(other, 0, Cell{ 0, 0 }).waypoints.size() == 4 );

  const auto path = pathFinder.findPath(agent, 100, Stages{ { 1, 0 }, { 2, 0 } });
  CHECK( cells(path) == std::vector<Cell>{ { 1, 0 }, { 2, 0 } } );
  CHECK( times(path) == std::vector<TimeInterval>{ { 100, 100 }, { 110, infinity } } );
}

TEST_CASE("PathFinder advance forgets finished waypoints, arcs and blockings") {
  auto pathFinder = GridWorld::makePathFinder({ 4, 1 });
  const PathFinder::AgentIndex agent = pathFinder.addAgent(0, { 0, 0 });
  pathFinder.updatePath(agent, 0, pathFinder.findPath(agent, 0, Cell{ 3, 0 }));

  // At time 15 the agent is on the arc from (1,0) to (2,0).
  pathFinder.advance(15);
  CHECK( pathFinder.now() == 15 );
  CHECK( cells(pathFinder.getPath(agent)) == std::vector<Cell>{ { 1, 0 }, { 2, 0 }, { 3, 0 } } );
  CHECK( pathFinder.getPath(agent).arcs.size() == 2 );
  CHECK( pathFinder.getBlockings().get(agent, { 0, 14 }).empty() );
  CHECK( !pathFinder.getBlockings().get(agent, { 15, 15 }).empty() );

  // At time 20 the agent arrives at (2,0), so the arc it arrives from is still occupied.
  pathFinder.advance(20);
  CHECK( cells(pathFinder.getPath(agent)) == std::vector<Cell>{ { 1, 0 }, { 2, 0 }, { 3, 0 } } );

  pathFinder.advance(21);
  CHECK( cells(pathFinder.getPath(agent)) == std::vector<Cell>{ { 2, 0 }, { 3, 0 } } );
}

TEST_CASE("PathFinder keeps planning after advance without blocking the past") {
  auto pathFinder = GridWorld::makePathFinder({ 4, 1 });
  const PathFinder::AgentIndex agent = pathFinder.addAgent(0, { 0, 0 });
  pathFinder.updatePath(agent, 0, pathFinder.findPath(agent, 0, Cell{ 3, 0 }));
  pathFinder.advance(25);

  const auto back = pathFinder.findPath(agent, 25, Cell{ 0, 0 });
  REQUIRE( cells(back) == std::vector<Cell>{ { 3, 0 }, { 2, 0 }, { 1, 0 }, { 0, 0 } } );
  CHECK( times(back).front() == TimeInterval{ 30, 30 } );

  pathFinder.updatePath(agent, 25, back);
  CHECK( cells(pathFinder.getPath(agent)) == std::vector<Cell>{ { 2, 0 }, { 3, 0 }, { 2, 0 }, { 1, 0 }, { 0, 0 } } );
  CHECK( pathFinder.getBlockings().get(agent, { 0, 24 }).empty() );
}

TEST_CASE("PathFinder finds equally cheap paths with a heuristic") {
  std::vector<TimeInterval> finalTimes;
  for ( bool guided : { false, true } ) {
    auto pathFinder = GridWorld::makePathFinder({ 5, 5, 1, {}, guided });
    const PathFinder::AgentIndex first = pathFinder.addAgent(0, { 2, 0 });
    const PathFinder::AgentIndex second = pathFinder.addAgent(0, { 0, 2 });
    pathFinder.updatePath(first, 0, pathFinder.findPath(first, 0, Cell{ 2, 4 }));

    const auto path = pathFinder.findPath(second, 0, Stages{ { 4, 0 }, { 0, 4 } });
    REQUIRE( !path.waypoints.empty() );
    finalTimes.push_back(times(path).back());
  }
  CHECK( finalTimes.front() == finalTimes.back() );
}

TEST_CASE("PathFinder drops labels from which the heuristic finds the target out of reach") {
  struct HopelessTraits : GridWorld::Traits {
    PathFinder::FixedPoint heuristic(PathFinder::AgentIndex, const Cell&, const Cell&) const { return infinity; }
  };
  PathFinder::PathFinder<HopelessTraits> pathFinder(HopelessTraits{ { 3, 1 } }, PathFinder::GridIndex<>(1.0));
  const PathFinder::AgentIndex agent = pathFinder.addAgent(0, { 0, 0 });

  CHECK( pathFinder.findPath(agent, 0, Cell{ 2, 0 }).waypoints.empty() );
  const auto staying = pathFinder.findPath(agent, 0, Cell{ 0, 0 });
  REQUIRE( staying.waypoints.size() == 1 );
  CHECK( staying.waypoints.front().node == Cell{ 0, 0 } );
}
