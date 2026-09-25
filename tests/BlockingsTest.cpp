#include "GridWorld.hpp"

#include <doctest/doctest.h>

#include <vector>

using PathFinder::Blocking;
using PathFinder::Rect;
using PathFinder::TimeInterval;
using GridWorld::Cell;
using GridWorld::Move;

namespace {

using Blockings = PathFinder::Blockings<GridWorld::Traits, PathFinder::GridIndex<>>;

const GridWorld::Traits traits{ 5, 5 };

Blocking<Rect> blockingAt(Cell cell, TimeInterval times) {
  return { times, traits.getBlockedArea(1, cell) };
}

} // namespace

TEST_CASE("Blockings remove trims blockings to the parts outside the interval") {
  Blockings blockings(traits, PathFinder::GridIndex<>(1.0));
  blockings.add(1, blockingAt({ 0, 0 }, { 0, 100 }));
  blockings.add(1, blockingAt({ 1, 0 }, { 0, 20 }));
  blockings.add(1, blockingAt({ 2, 0 }, { 60, 80 }));
  blockings.add(1, blockingAt({ 3, 0 }, { 40, 50 }));

  blockings.remove(1, TimeInterval{ 30, 70 });

  CHECK( blockings.get(1, { 0, TimeInterval::infinity }) == std::vector<Blocking<Rect>>{
    blockingAt({ 0, 0 }, { 0, 29 }),
    blockingAt({ 1, 0 }, { 0, 20 }),
    blockingAt({ 2, 0 }, { 71, 80 }),
    blockingAt({ 0, 0 }, { 71, 100 })
  } );
}

TEST_CASE("Blockings remove of a blocking and clear") {
  Blockings blockings(traits, PathFinder::GridIndex<>(1.0));
  blockings.add(1, blockingAt({ 0, 0 }, { 0, 10 }));
  blockings.add(1, blockingAt({ 1, 0 }, { 0, 10 }));

  blockings.remove(1, blockingAt({ 0, 0 }, { 0, 10 }));
  CHECK( blockings.get(1, { 0, TimeInterval::infinity }) == std::vector<Blocking<Rect>>{ blockingAt({ 1, 0 }, { 0, 10 }) } );

  blockings.clear(1);
  CHECK( blockings.get(1, { 0, TimeInterval::infinity }).empty() );
  CHECK( blockings.availabilities(2, { 0, 100 }, Cell{ 1, 0 }) == std::vector<TimeInterval>{ { 0, 100 } } );
}

TEST_CASE("Blockings availabilities of a node ignore own blockings and touching areas") {
  Blockings blockings(traits, PathFinder::GridIndex<>(1.0));
  blockings.add(1, blockingAt({ 1, 1 }, { 10, 20 }));
  blockings.add(1, blockingAt({ 1, 1 }, { 15, 30 }));
  blockings.add(1, blockingAt({ 1, 1 }, { 50, TimeInterval::infinity }));
  blockings.add(1, blockingAt({ 2, 1 }, { 0, TimeInterval::infinity }));

  CHECK( blockings.availabilities(2, { 0, 100 }, Cell{ 1, 1 }) == std::vector<TimeInterval>{ { 0, 9 }, { 31, 49 } } );
  CHECK( blockings.availabilities(2, { 12, 40 }, Cell{ 1, 1 }) == std::vector<TimeInterval>{ { 31, 40 } } );
  CHECK( blockings.availabilities(1, { 0, 100 }, Cell{ 1, 1 }) == std::vector<TimeInterval>{ { 0, 100 } } );
}

TEST_CASE("Blockings availabilities of an arc include its origin and destination") {
  Blockings blockings(traits, PathFinder::GridIndex<>(1.0));
  blockings.add(1, blockingAt({ 0, 0 }, { 0, 10 }));
  blockings.add(1, blockingAt({ 1, 0 }, { 30, 40 }));

  CHECK( blockings.availabilities(2, { 0, TimeInterval::infinity }, Move{ { 0, 0 }, { 1, 0 } }) ==
         std::vector<TimeInterval>{ { 11, 29 }, { 41, TimeInterval::infinity } } );
}
