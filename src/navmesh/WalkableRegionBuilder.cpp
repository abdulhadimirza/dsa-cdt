#include "navmesh/WalkableRegionBuilder.hpp"
#include "clipper2/clipper.h"
#include <cmath>
#include <limits>
#include <utility>

namespace cdt::detail {
struct WalkableRegionSetBuilder {
    // Keep construction private to the builder pipeline so arbitrary unsanitized regions cannot
    // cross the type boundary into constrained triangulation.
    static WalkableRegionSet Create(std::vector<WalkableRegion> regions) {
        return WalkableRegionSet(std::move(regions));
    }
};
namespace {
// Convert the immutable input loop to Clipper's path type without changing point order.
Clipper2Lib::PathD ToClipper(const ValidatedLoop& loop) {
    Clipper2Lib::PathD path;
    path.reserve(loop.vertices.size());
    for (const Vector2d& p : loop.vertices) path.emplace_back(p.x, p.y);
    return path;
}
// Convert Clipper output back to the project's point representation.
std::vector<Vector2d> ToVector(const Clipper2Lib::PathD& path) {
    std::vector<Vector2d> out;
    out.reserve(path.size());
    for (const auto& p : path) out.emplace_back(p.x, p.y);
    return out;
}
// Reject contours that cannot form a simple polygon input: non-finite points, short edges,
// repeated vertices, or fewer than three distinct vertices.
bool Usable(const std::vector<Vector2d>& contour) {
    if (contour.size() < 3) return false;
    for (std::size_t i = 0; i < contour.size(); ++i) {
        const auto& p = contour[i];
        const auto& q = contour[(i + 1) % contour.size()];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p == q) return false;
        for (std::size_t j = i + 1; j < contour.size(); ++j)
            if (p == contour[j]) return false;
    }
    return true;
}
// Clipper can produce duplicate points across distinct contours; reject these before meshing
// because they would give separate boundaries ambiguous shared vertex identity.
bool HasCoincidentOutputPoints(const Clipper2Lib::PathsD& contours) {
    std::vector<Vector2d> points;
    for (const auto& contour : contours)
        for (const auto& point : contour) {
            const Vector2d candidate{point.x, point.y};
            for (const Vector2d& existing : points)
                if (candidate == existing) return true;
            points.push_back(candidate);
        }
    return false;
}
// Preserve IDs only for exact points that survive offsetting unchanged; generated offset points
// intentionally retain null identity.
std::vector<std::optional<std::uint64_t>> SourceIds(
    const std::vector<Vector2d>& contour, const ValidatedNavmeshInput& geometry) {
    std::vector<std::optional<std::uint64_t>> ids(contour.size());
    auto match = [&](const ValidatedLoop& source) {
        for (std::size_t i = 0; i < contour.size(); ++i)
            for (std::size_t j = 0; j < source.vertices.size(); ++j)
                if (contour[i] == source.vertices[j]) ids[i] = source.vertex_ids[j];
    };
    match(geometry.ActiveBorder());
    for (const auto& obstacle : geometry.EligibleObstacles()) match(obstacle);
    return ids;
}
}

std::optional<WalkableRegionSet> BuildWalkableRegions(
    const ValidatedNavmeshInput& geometry, double agentRadius, std::string& error) {
    using namespace Clipper2Lib;
    if (!std::isfinite(agentRadius) || agentRadius < 0.0) {
        error = "Agent radius must be a finite, nonnegative world-space value.";
        return std::nullopt;
    }
    constexpr double roundJoinToleranceFraction = 0.20;
    const double roundJoinTolerance = agentRadius * roundJoinToleranceFraction;
    // Shrink the border by the agent radius and expand obstacles by the same amount, then subtract
    // expanded obstacles from the inset border to produce the actual walkable space.
    const PathsD inset = InflatePaths(PathsD{ToClipper(geometry.ActiveBorder())}, -agentRadius,
        JoinType::Round, EndType::Polygon, 2.0, 8, roundJoinTolerance);
    if (inset.empty()) { error = "Agent radius removes the walkable border."; return std::nullopt; }
    PathsD expanded;
    for (const auto& obstacle : geometry.EligibleObstacles()) {
        auto offset = InflatePaths(PathsD{ToClipper(obstacle)}, agentRadius,
            JoinType::Round, EndType::Polygon, 2.0, 8, roundJoinTolerance);
        expanded.insert(expanded.end(), offset.begin(), offset.end());
    }
    if (!expanded.empty()) expanded = Union(expanded, FillRule::NonZero, 8);
    const PathsD contours = Difference(inset, expanded, FillRule::NonZero, 8);
    if (contours.empty()) { error = "Agent radius leaves no walkable area."; return std::nullopt; }
    if (HasCoincidentOutputPoints(contours)) {
        error = "Offset contours contain coincident vertices.";
        return std::nullopt;
    }
    for (const auto& contour : contours) {
        const auto points = ToVector(contour);
        const double area = Area(contour);
        if (!Usable(points) || !std::isfinite(area) || area == 0.0) {
            error = "Offset contours contain invalid or coincident vertices.";
            return std::nullopt;
        }
    }
    // Clipper's signed area distinguishes outer contours from holes; assign each hole to the
    // containing outer boundary before publishing the typed region set.
    std::vector<WalkableRegion> regions;
    for (const auto& outer : contours) {
        if (Area(outer) <= 0.0) continue;
        WalkableRegion region;
        region.outerContour = ToVector(outer);
        if (!Usable(region.outerContour) ||
            std::abs(Area(outer)) <= std::numeric_limits<double>::epsilon()) {
            error = "Offset contours contain invalid or coincident vertices.";
            return std::nullopt;
        }
        region.outerSourceIds = SourceIds(region.outerContour, geometry);
        for (const auto& contour : contours) {
            if (Area(contour) >= 0.0 || contour.empty()) continue;
            if (PointInPolygon(contour.front(), outer) == PointInPolygonResult::IsInside) {
                auto hole = ToVector(contour);
                if (!Usable(hole)) {
                    error = "Offset contours contain invalid or coincident vertices.";
                    return std::nullopt;
                }
                region.holeSourceIds.push_back(SourceIds(hole, geometry));
                region.holeContours.push_back(std::move(hole));
            }
        }
        regions.push_back(std::move(region));
    }
    if (regions.empty()) {
        error = "Agent radius leaves no triangulatable walkable area.";
        return std::nullopt;
    }
    return WalkableRegionSetBuilder::Create(std::move(regions));
}
}

