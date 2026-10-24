#include "pathfinding/Pathfinder.hpp"
#include "core/GeometryPredicates.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <tuple>
#include <utility>

namespace cdt::pathfinding {
namespace {
// Portal endpoints are stored in travel-left/travel-right order for funnel tightening.
struct Portal { Vector2d left; Vector2d right; };

/// Returns a triangle's centroid for corridor-search edge costs and heuristic estimates.
Vector2d Centroid(const NavMesh& mesh, const MeshTriangle& triangle) {
    const auto& vertices = mesh.Vertices();
    return (vertices[triangle.vertices[0]].position + vertices[triangle.vertices[1]].position +
        vertices[triangle.vertices[2]].position) / 3.0;
}

/// Tests a point against a counterclockwise triangle, including points within edge tolerance.
bool ContainsPoint(const NavMesh& mesh, const MeshTriangle& triangle, Vector2d point) {
    const auto& vertices = mesh.Vertices();
    const Vector2d a = vertices[triangle.vertices[0]].position;
    const Vector2d b = vertices[triangle.vertices[1]].position;
    const Vector2d c = vertices[triangle.vertices[2]].position;
    constexpr double edgeTolerance = 1e-10;
    return geometry::Cross2D(b - a, point - a) >= -edgeTolerance &&
        geometry::Cross2D(c - b, point - b) >= -edgeTolerance &&
        geometry::Cross2D(a - c, point - c) >= -edgeTolerance;
}

/// Returns the first traversable triangle containing the point, if any.
std::optional<std::size_t> FindContainingTriangle(const NavMesh& mesh, Vector2d point) {
    for (std::size_t i = 0; i < mesh.Triangles().size(); ++i) {
        const auto& triangle = mesh.Triangles()[i];
        if (triangle.region == TriangleRegion::Traversable && ContainsPoint(mesh, triangle, point)) return i;
    }
    return std::nullopt;
}

/// Finds a minimum-cost traversable triangle corridor with A*; returns empty when disconnected.
std::optional<std::vector<std::size_t>> FindCorridor(
    const NavMesh& mesh, std::size_t start, std::size_t goal) {
    const std::size_t count = mesh.Triangles().size();
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> costs(count, infinity);
    std::vector<std::size_t> previous(count, count);
    using Entry = std::tuple<double, double, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> pending;
    const Vector2d goalCenter = Centroid(mesh, mesh.Triangles()[goal]);
    costs[start] = 0.0;
    pending.emplace(Centroid(mesh, mesh.Triangles()[start]).distance(goalCenter), 0.0, start);
    // Costs are centroid-to-centroid distances, so distance to the goal centroid is admissible.
    while (!pending.empty()) {
        const auto [estimate, cost, current] = pending.top();
        pending.pop();
        (void)estimate;
        if (cost > costs[current]) continue;
        if (current == goal) break;
        const Vector2d currentCenter = Centroid(mesh, mesh.Triangles()[current]);
        for (const auto neighbor : mesh.Triangles()[current].neighbors) {
            if (!neighbor || mesh.Triangles()[*neighbor].region != TriangleRegion::Traversable) continue;
            const Vector2d center = Centroid(mesh, mesh.Triangles()[*neighbor]);
            const double candidate = costs[current] + currentCenter.distance(center);
            if (candidate < costs[*neighbor]) {
                costs[*neighbor] = candidate;
                previous[*neighbor] = current;
                pending.emplace(candidate + center.distance(goalCenter), candidate, *neighbor);
            }
        }
    }
    if (start != goal && previous[goal] == count) return std::nullopt;
    std::vector<std::size_t> corridor;
    for (std::size_t current = goal;; current = previous[current]) {
        corridor.push_back(current);
        if (current == start) break;
    }
    std::reverse(corridor.begin(), corridor.end());
    return corridor;
}

/// Converts consecutive corridor faces into shared-edge portals ordered for funnel smoothing.
std::vector<Portal> BuildPortals(const NavMesh& mesh, const std::vector<std::size_t>& corridor,
    Vector2d start, Vector2d goal) {
    std::vector<Portal> portals{{start, start}};
    for (std::size_t i = 0; i + 1 < corridor.size(); ++i) {
        const auto current = corridor[i], next = corridor[i + 1];
        std::size_t slot = 0;
        while (mesh.Triangles()[current].neighbors[slot] != next) ++slot;
        const auto& triangle = mesh.Triangles()[current];
        const auto& vertices = mesh.Vertices();
        const Vector2d a = vertices[triangle.vertices[(slot + 1) % 3]].position;
        const Vector2d b = vertices[triangle.vertices[(slot + 2) % 3]].position;
        // Use the opposite-edge order expected by SmoothPath's signed-area tests.
        portals.push_back({a, b});
    }
    portals.push_back({goal, goal});
    return portals;
}

/// Tightens funnel bounds and emits corridor corners when the bounds cross.
std::vector<Vector2d> SmoothPath(const std::vector<Portal>& portals, Vector2d start, Vector2d goal) {
    std::vector<Vector2d> waypoints{start};
    Vector2d tip = start, left = portals[0].left, right = portals[0].right;
    std::size_t tipIndex = 0, leftIndex = 0, rightIndex = 0;
    for (std::size_t i = 1; i < portals.size(); ++i) {
        const Vector2d newLeft = portals[i].left, newRight = portals[i].right;
        if (geometry::Cross2D(right - tip, newRight - tip) <= 0.0) {
            if (tip == right || geometry::Cross2D(left - tip, newRight - tip) > 0.0) {
                right = newRight; rightIndex = i;
            } else {
                waypoints.push_back(left);
                tip = left; tipIndex = leftIndex;
                left = right = tip; leftIndex = rightIndex = tipIndex;
                i = tipIndex;
                continue;
            }
        }
        if (geometry::Cross2D(left - tip, newLeft - tip) >= 0.0) {
            if (tip == left || geometry::Cross2D(right - tip, newLeft - tip) < 0.0) {
                left = newLeft; leftIndex = i;
            } else {
                waypoints.push_back(right);
                tip = right; tipIndex = rightIndex;
                left = right = tip; leftIndex = rightIndex = tipIndex;
                i = tipIndex;
            }
        }
    }
    if (waypoints.back() != goal) waypoints.push_back(goal);
    return waypoints;
}
}

/// Locates both endpoints, finds their traversable corridor, and returns its smoothed route.
PathResult FindPath(const NavMesh& mesh, Vector2d start, Vector2d goal) {
    PathResult result;
    const auto startTriangle = FindContainingTriangle(mesh, start);
    if (!startTriangle) { result.status = PathStatus::StartOutsideMesh; return result; }
    const auto goalTriangle = FindContainingTriangle(mesh, goal);
    if (!goalTriangle) { result.status = PathStatus::GoalOutsideMesh; return result; }
    auto corridor = FindCorridor(mesh, *startTriangle, *goalTriangle);
    if (!corridor) { result.status = PathStatus::NoPath; return result; }
    result.triangleCorridor = std::move(*corridor);
    result.waypoints = SmoothPath(BuildPortals(mesh, result.triangleCorridor, start, goal), start, goal);
    result.status = PathStatus::Success;
    return result;
}
}
