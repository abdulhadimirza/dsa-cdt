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

} // namespace cdt::detail
