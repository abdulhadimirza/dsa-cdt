#pragma once

#include "core/Vector2d.hpp"
#include <span>
#include <cstddef>

namespace cdt::geometry {

/// Signed 2D cross product of vectors from a common origin.
[[nodiscard]] double Cross2D(Vector2d a, Vector2d b) noexcept;
/// Robust signed orientation; positive means counterclockwise.
[[nodiscard]] double Orient2D(Vector2d a, Vector2d b, Vector2d c);
/// Robust in-circle predicate for a counterclockwise triangle.
[[nodiscard]] double InCircle2D(Vector2d a, Vector2d b, Vector2d c, Vector2d d);
/// Signed twice-area, accumulated relative to the first vertex to reduce cancellation.
[[nodiscard]] double SignedTwiceArea(std::span<const Vector2d> polygon) noexcept;

/// Controls whether PointInPolygon treats points on an edge as contained.
enum class BoundaryPolicy {
    Inside, ///< Boundary points count as contained.
    Outside ///< Boundary points do not count as contained.
};
/// Ray-cast polygon containment with an explicit result for boundary points.
[[nodiscard]] bool PointInPolygon(std::span<const Vector2d> polygon, Vector2d point,
    BoundaryPolicy boundaryPolicy);

} // namespace cdt::geometry
