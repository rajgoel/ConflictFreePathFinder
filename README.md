# ConflictFreePathFinder

A header-only C++20 library for collaborative path finding with deadlock avoidance. Agents are planned one at a
time: each search finds the cheapest path for one agent that avoids everything the other agents have committed
to, in space and in time, and a committed path ends at a node the agent may occupy forever, so that no agent is
ever left without a way to continue. The approach follows Möhring et al., *Conflict-free Real-time AGV Routing*.

The library does not constrain the caller's types. The graph, the agents and the space they occupy are described
by a traits class, and nodes, arcs and areas can be whatever the caller already has.

## Concepts

- **Blockings.** An agent occupies an area while it rests at a node or travels along an arc. A committed path
  blocks the area of a node from the departure towards it until the arrival at the next node, and the area of an
  arc while it is traversed. Areas are compared with `overlaps`; the default area is an axis-aligned rectangle.
- **Time windows.** The search is label-setting with time windows (closed intervals of fixed-point time): a label
  holds the times the agent can be at a node, given everything committed. Waiting is allowed where the traits say
  so and costs `waitingCost` per time unit.
- **Stages.** A path visits a sequence of nodes in order, for instance a pickup, a drop-off and a parking position.
  The final node must be free forever, which is what keeps the agents free of deadlocks.
- **Heuristic.** An optional lower bound of the costs from a node to a target guides the search (A*). The search
  adds the bounds between the later stages, so that stages still to come count before they are reached.

## Traits

```cpp
struct Traits {
  using Node = ...;                 // equality comparable and hashable with std::hash
  using Arc = ...;
  using Area = ...;                 // optional, defaults to PathFinder::Rect

  Node origin(const Arc&) const;
  Node destination(const Arc&) const;
  auto outgoing(AgentIndex, const Node&) const;      // an input range of arcs
  FixedPoint duration(AgentIndex, const Arc&) const;
  FixedPoint costs(AgentIndex, const Arc&) const;
  bool canWait(AgentIndex, const Node&) const;
  FixedPoint waitingCost(AgentIndex) const;
  Area getBlockedArea(AgentIndex, const Arc&) const;
  Area getBlockedArea(AgentIndex, const Node&) const;

  // Optional
  FixedPoint heuristic(AgentIndex, const Node&, const Node& target) const;  // defaults to 0
  bool overlaps(const Area&, const Area&) const;                          // defaults to a free overlaps() found by ADL
};
```

Costs must be at least `waitingCost * duration` for every arc, so that driving in circles is never cheaper than
waiting. A heuristic may return `TimeInterval::infinity` where the target cannot be reached from a node.

## Usage

```cpp
#include <PathFinder.hpp>

#include <cstdlib>
#include <iostream>
#include <vector>

// A corridor of cells 0..9; every move to a neighbour takes 10 time units and costs 10.
struct Move {
  int from;
  int to;
};

struct Corridor {
  using Node = int;
  using Arc = Move;

  Node origin(const Arc& arc) const { return arc.from; }
  Node destination(const Arc& arc) const { return arc.to; }

  std::vector<Arc> outgoing(PathFinder::AgentIndex, Node node) const {
    std::vector<Arc> arcs;
    for ( Node next : { node - 1, node + 1 } ) {
      if ( next >= 0 && next < 10 ) {
        arcs.push_back({ node, next });
      }
    }
    return arcs;
  }

  PathFinder::FixedPoint duration(PathFinder::AgentIndex, const Arc&) const { return 10; }
  PathFinder::FixedPoint costs(PathFinder::AgentIndex, const Arc&) const { return 10; }
  bool canWait(PathFinder::AgentIndex, Node) const { return true; }
  PathFinder::FixedPoint waitingCost(PathFinder::AgentIndex) const { return 1; }

  // Optional: a lower bound of the costs to the target.
  PathFinder::FixedPoint heuristic(PathFinder::AgentIndex, Node node, Node target) const {
    return 10 * std::abs(target - node);
  }

  // The floor an agent holds, here a unit square around each cell it is on.
  PathFinder::Rect getBlockedArea(PathFinder::AgentIndex, const Arc& arc) const {
    return { std::min(arc.from, arc.to) - 0.5, -0.5, std::max(arc.from, arc.to) + 0.5, 0.5 };
  }
  PathFinder::Rect getBlockedArea(PathFinder::AgentIndex, Node node) const {
    return { node - 0.5, -0.5, node + 0.5, 0.5 };
  }
};

int main() {
  PathFinder::PathFinder<Corridor> pathFinder(Corridor{}, PathFinder::GridIndex<>(1.0));
  const PathFinder::AgentIndex first = pathFinder.addAgent(0, 0);
  const PathFinder::AgentIndex second = pathFinder.addAgent(0, 9);

  // The first agent drives to cell 5 and stays there.
  pathFinder.updatePath(first, 0, pathFinder.findPath(first, 0, 5));

  // The second agent fetches something at cell 7 and goes back to cell 9.
  const auto path = pathFinder.findPath(second, 0, { 7, 9 });
  for ( const auto& waypoint : path.waypoints ) {
    std::cout << waypoint.node << " from " << waypoint.times.begin << '\n';
  }
  pathFinder.updatePath(second, 0, path);
}
```

## API

`PathFinder::PathFinder<Traits, Index = GridIndex<Area>>`

| Member | |
|---|---|
| `AgentIndex addAgent(FixedPoint time, const Node& node)` | Adds an agent resting at `node` from `time` on; returns 0 if the node is not free forever. |
| `Path findPath(AgentIndex, FixedPoint time, const Stages& stages)` | The cheapest path from where the agent is at `time` through the nodes of `stages` in order, ending at a node free forever; empty if there is none. Commits nothing. |
| `Path findPath(AgentIndex, FixedPoint time, const Node& node)` | The same for a single stage. |
| `void updatePath(AgentIndex, FixedPoint time, const Path&)` | Commits a path found for the same agent and time from `time` on. |
| `void clearPath(AgentIndex)` | Removes the agent's path, which then blocks nothing. |
| `const Path& getPath(AgentIndex)` | The agent's committed path. |
| `FixedPoint now()` / `void advance(FixedPoint time)` | The current time, and moving it forward, forgetting what has finished. |
| `const Blockings& getBlockings()` | Everything committed, by agent. |

A `Path` holds the waypoints, each a node with the interval from arrival to departure, and the arcs between them.
Time is a 64-bit fixed-point number in whatever unit the traits use.

The spatial index used to find overlapping blockings is a customization point: any type with `insert`, `erase` and
`forEachCandidate` will do (see `SpatialIndex.hpp`). The default is a uniform grid over the areas' bounding boxes.

## Building

The library is header only. With CMake:

```cmake
add_subdirectory(ConflictFreePathFinder)
target_link_libraries(your_target PRIVATE PathFinder::PathFinder)
```

The tests use doctest, fetched by CMake:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## License

MIT, see [LICENSE](LICENSE).
