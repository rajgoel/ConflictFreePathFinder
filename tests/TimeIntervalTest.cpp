#include "path_finder/TimeInterval.hpp"

#include <doctest/doctest.h>

using PathFinder::TimeInterval;

TEST_CASE("TimeInterval is closed") {
  const TimeInterval interval{ 10, 20 };

  CHECK( interval.contains(10) );
  CHECK( interval.contains(20) );
  CHECK_FALSE( interval.contains(21) );
  CHECK( interval.overlaps(TimeInterval{ 20, 30 }) );
  CHECK_FALSE( interval.overlaps(TimeInterval{ 21, 30 }) );
}

TEST_CASE("TimeInterval of a single instant is not empty") {
  CHECK_FALSE( TimeInterval{ 10, 10 }.empty() );
  CHECK( TimeInterval{ 10, 10 }.contains(10) );
  CHECK( TimeInterval{ 10, 9 }.empty() );
}

TEST_CASE("TimeInterval intersection") {
  CHECK( TimeInterval{ 0, 20 }.intersection(TimeInterval{ 10, 30 }) == TimeInterval{ 10, 20 } );
  CHECK( TimeInterval{ 0, 10 }.intersection(TimeInterval{ 10, 30 }) == TimeInterval{ 10, 10 } );
  CHECK( TimeInterval{ 0, 10 }.intersection(TimeInterval{ 20, 30 }).empty() );
}

TEST_CASE("TimeInterval shifted keeps an unbounded end unbounded") {
  CHECK( TimeInterval{ 10, 20 }.shifted(5) == TimeInterval{ 15, 25 } );
  CHECK( TimeInterval{ 10, TimeInterval::infinity }.shifted(5) == TimeInterval{ 15, TimeInterval::infinity } );
}
