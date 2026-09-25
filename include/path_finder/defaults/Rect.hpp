#pragma once

namespace PathFinder {

/// Axis-aligned 2D rectangle, the default area type.
struct Rect {
  double minX = 0.0;
  double minY = 0.0;
  double maxX = 0.0;
  double maxY = 0.0;

  friend constexpr bool operator==(const Rect&, const Rect&) = default;
};

/// Rectangles overlap if their interiors intersect; touching edges do not count as overlap.
constexpr bool overlaps(const Rect& rect, const Rect& otherRect) {
  return rect.minX < otherRect.maxX && otherRect.minX < rect.maxX &&
         rect.minY < otherRect.maxY && otherRect.minY < rect.maxY;
}

/// Bounding box used by the default spatial index. Custom area types provide their own `boundingBox` found by ADL.
constexpr const Rect& boundingBox(const Rect& rect) {
  return rect;
}

} // namespace PathFinder
