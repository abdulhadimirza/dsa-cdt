#pragma once

#include "navmesh/WalkableRegion.hpp"
#include "validation/GeometryValidator.hpp"

#include <string>
#include <optional>
#include <utility>
#include <vector>

namespace cdt::detail {

/// Offset a validated snapshot and sanitize Clipper output; failure explains unusable contours.
[[nodiscard]] std::optional<WalkableRegionSet> BuildWalkableRegions(
    const ValidatedNavmeshInput& geometry, double agentRadius, std::string& error);

/// Compute the exact Clipper symmetric difference between two already-built walkable region sets.
[[nodiscard]] std::optional<WalkableRegionDelta> DiffWalkableRegions(
    const WalkableRegionSet& previous, const WalkableRegionSet& current, std::string& error);

/// Clip walkable contours to a closed cavity boundary and return builder-sanitized contours.
[[nodiscard]] std::optional<WalkableRegionSet> ClipWalkableRegionsToCavity(
    const WalkableRegionSet& walkableRegions, const std::vector<std::vector<Vector2d>>& cavityLoops,
    std::string& error);

} // namespace cdt::detail
