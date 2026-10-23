#pragma once

#include "model/GeometryModel.hpp"
#include "validation/GeometryValidator.hpp"

#include <string>
#include <vector>

namespace cdt::detail {

/// One connected walkable area: its enclosing boundary and any excluded holes.
struct WalkableRegion {
    std::vector<Vector2d> outerContour;
    std::vector<std::vector<Vector2d>> holeContours;
};

/// Build agent-radius-adjusted regions from the validated border and obstacle loops.
/// On failure, returns false and writes a user-facing explanation to `error`.
bool BuildWalkableRegions(const GeometryModel& model, const GeometryValidation& validation,
    double agentRadius, std::vector<WalkableRegion>& regions, std::string& error);

} // namespace cdt::detail
