#pragma once

#include "PathFinder.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstddef>
#include <functional>
#include <vector>

namespace GridWorld {

struct Cell {
  int x;
  int y;

  friend bool operator==(const Cell&, const Cell&) = default;
};

struct Move {
  Cell from;
  Cell to;
};

} // namespace GridWorld

template<>
struct std::hash<GridWorld::Cell> {
  std::size_t operator()(const GridWorld::Cell& cell) const {
    return std::hash<int>{}(cell.x) * 31 + std::hash<int>{}(cell.y);
  }
};

namespace GridWorld {

/// 4-connected grid of unit cells. Every move takes 10 time units and costs 10.
struct Traits {
  using Node = Cell;
  using Arc = Move;

  int width;
  int height;
  PathFinder::FixedPoint waitingCostPerTime = 1;
  std::vector<Cell> noWaitCells = {};
  bool guided = false;

  Cell origin(const Move& move) const { return move.from; }
  Cell destination(const Move& move) const { return move.to; }

  std::vector<Move> outgoing(PathFinder::AgentIndex, const Cell& cell) const {
    std::vector<Move> moves;
    for ( const Cell& neighbour : { Cell{ cell.x + 1, cell.y }, Cell{ cell.x - 1, cell.y }, Cell{ cell.x, cell.y + 1 }, Cell{ cell.x, cell.y - 1 } } ) {
      if ( neighbour.x >= 0 && neighbour.x < width && neighbour.y >= 0 && neighbour.y < height ) {
        moves.push_back({ cell, neighbour });
      }
    }
    return moves;
  }

  PathFinder::FixedPoint duration(PathFinder::AgentIndex, const Move&) const { return 10; }
  PathFinder::FixedPoint costs(PathFinder::AgentIndex, const Move&) const { return 10; }
  bool canWait(PathFinder::AgentIndex, const Cell& cell) const { return std::ranges::find(noWaitCells, cell) == noWaitCells.end(); }
  PathFinder::FixedPoint waitingCost(PathFinder::AgentIndex) const { return waitingCostPerTime; }

  /// Ten per step to the target if guided, which never overestimates the costs.
  PathFinder::FixedPoint heuristic(PathFinder::AgentIndex, const Cell& cell, const Cell& target) const {
    if ( !guided ) {
      return 0;
    }
    return 10 * ( std::abs(target.x - cell.x) + std::abs(target.y - cell.y) );
  }

  PathFinder::Rect getBlockedArea(PathFinder::AgentIndex, const Cell& cell) const {
    return { cell.x - 0.5, cell.y - 0.5, cell.x + 0.5, cell.y + 0.5 };
  }

  PathFinder::Rect getBlockedArea(PathFinder::AgentIndex, const Move& move) const {
    return {
      std::min(move.from.x, move.to.x) - 0.5,
      std::min(move.from.y, move.to.y) - 0.5,
      std::max(move.from.x, move.to.x) + 0.5,
      std::max(move.from.y, move.to.y) + 0.5
    };
  }
};

using PathFinder = ::PathFinder::PathFinder<Traits>;

inline PathFinder makePathFinder(Traits traits) {
  return PathFinder(std::move(traits), ::PathFinder::GridIndex<>(1.0));
}

} // namespace GridWorld
