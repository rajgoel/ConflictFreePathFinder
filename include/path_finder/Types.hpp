#pragma once

#include <cstdint>

namespace PathFinder {

/// Agents are numbered from 1; 0 denotes an invalid agent.
using AgentIndex = unsigned int;

/// Fixed-point value for times and costs. The unit (e.g. milliseconds) is chosen by the user.
/// Integer arithmetic keeps comparisons exact and results deterministic across platforms.
using FixedPoint = std::int64_t;

} // namespace PathFinder
