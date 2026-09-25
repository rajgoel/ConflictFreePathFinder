#include "path_finder/defaults/Rect.hpp"

#include <doctest/doctest.h>

using PathFinder::Rect;

TEST_CASE("Rect overlaps") {
  const Rect rect{ 0.0, 0.0, 2.0, 2.0 };

  CHECK( overlaps(rect, Rect{ 1.0, 1.0, 3.0, 3.0 }) );
  CHECK( overlaps(rect, Rect{ 0.5, 0.5, 1.5, 1.5 }) );
  CHECK_FALSE( overlaps(rect, Rect{ 2.0, 0.0, 4.0, 2.0 }) );
  CHECK_FALSE( overlaps(rect, Rect{ 0.0, 3.0, 2.0, 4.0 }) );
}
