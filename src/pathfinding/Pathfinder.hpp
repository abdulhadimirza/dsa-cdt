#pragma once
#include "cdt/ConstrainedDelaunay.hpp"
namespace cdt::pathfinding {
/// Outcome of endpoint location and traversal through the generated traversable mesh.
enum class PathStatus { Success, StartOutsideMesh, GoalOutsideMesh, NoPath };
/// Triangle corridor and smoothed world-space route for one path query.
struct PathResult {
    /// Distinguishes successful routes from endpoints outside or disconnected regions.
    PathStatus status{PathStatus::NoPath};
    /// Ordered traversable triangle indices from start face through goal face.
    std::vector<std::size_t> triangleCorridor;
    /// World-space corners beginning at start and ending at goal on success.
    std::vector<Vector2d> waypoints;
};
/// Locate both endpoints, search traversable adjacency, and funnel-smooth the resulting corridor.
/// NavMesh guarantees valid triangle and neighbor indices from successful generation.
[[nodiscard]] PathResult FindPath(const NavMesh& mesh, Vector2d start, Vector2d goal);
}
