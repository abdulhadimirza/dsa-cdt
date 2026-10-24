#pragma once

#include "core/Vector2d.hpp"
#include "pathfinding/Pathfinder.hpp"

#include <optional>

namespace cdt::pathfinding {

/// Non-owning, read-only presentation data; references remain valid for the owner's lifetime.
struct PathfindingView {
    /// Optional world-space start point.
    const std::optional<Vector2d>& start;
    /// Optional world-space goal point.
    const std::optional<Vector2d>& goal;
    /// Latest generated result, or empty while no result is current.
    const std::optional<PathResult>& result;
};

} // namespace cdt::pathfinding
