#include "cdt/WalkableRegionBuilder.hpp"

#include "clipper2/clipper.h"

#include <cmath>
#include <cstddef>
#include <utility>

namespace cdt::detail {
namespace {

bool MakeSourceContour(const GeometryModel& model, const GeometryLoop& loop, Clipper2Lib::PathD& contour) {
    for (VertexId id : loop.vertices) {
        const Vertex* vertex = model.FindVertex(id);
        if (!vertex || !std::isfinite(vertex->position.x) || !std::isfinite(vertex->position.y)) return false;
        contour.emplace_back(vertex->position.x, vertex->position.y);
    }
    return contour.size() >= 3;
}

std::vector<Vector2d> ToVectorContour(const Clipper2Lib::PathD& contour) {
    std::vector<Vector2d> result;
    result.reserve(contour.size());
    for (const Clipper2Lib::PointD& point : contour) result.push_back({point.x, point.y});
    return result;
}

} // namespace

bool BuildWalkableRegions(const GeometryModel& model, const GeometryValidation& validation,
    double agentRadius, std::vector<WalkableRegion>& regions, std::string& error) {
    using namespace Clipper2Lib;
    regions.clear();

    if (!validation.active_border_index) {
        error = "No valid active border.";
        return false;
    }
    const std::size_t borderIndex = *validation.active_border_index;
    if (borderIndex >= model.Loops().size() || borderIndex >= validation.loops.size() ||
        !validation.loops[borderIndex].active_border) {
        error = "Active border input is inconsistent.";
        return false;
    }

    // Offset the border inward and obstacles outward, then union and subtract them to find free contours.
    constexpr double roundJoinToleranceFraction = 0.20;
    const double roundJoinTolerance = agentRadius * roundJoinToleranceFraction;
    PathD borderContour;
    if (!MakeSourceContour(model, model.Loops()[borderIndex], borderContour)) {
        error = "Active border input is inconsistent.";
        return false;
    }
    PathsD insetBorderContours = InflatePaths(PathsD{borderContour}, -agentRadius,
        JoinType::Round, EndType::Polygon, 2.0, 8, roundJoinTolerance);
    if (insetBorderContours.empty()) {
        error = "Agent radius removes the walkable border.";
        return false;
    }

    PathsD expandedObstacleContours;
    for (std::size_t loopIndex = 0; loopIndex < model.Loops().size(); ++loopIndex) {
        if (loopIndex == borderIndex || loopIndex >= validation.loops.size() ||
            !validation.loops[loopIndex].eligible || model.Loops()[loopIndex].type != EdgeType::Obstacle) continue;
        PathD obstacleContour;
        if (!MakeSourceContour(model, model.Loops()[loopIndex], obstacleContour)) {
            error = "Obstacle input is inconsistent.";
            return false;
        }
        PathsD expandedContours = InflatePaths(PathsD{obstacleContour}, agentRadius,
            JoinType::Round, EndType::Polygon, 2.0, 8, roundJoinTolerance);
        expandedObstacleContours.insert(expandedObstacleContours.end(), expandedContours.begin(), expandedContours.end());
    }

    if (!expandedObstacleContours.empty())
        expandedObstacleContours = Union(expandedObstacleContours, FillRule::NonZero, 8);
    PathsD walkableContours = Difference(insetBorderContours, expandedObstacleContours, FillRule::NonZero, 8);
    if (walkableContours.empty()) {
        error = "Agent radius leaves no walkable area.";
        return false;
    }

    // Clipper orients outer contours with positive area and holes with negative area. Test one
    // point from each hole against each outer contour so every region receives only its own holes.
    for (const PathD& outerContour : walkableContours) {
        if (Area(outerContour) <= 0.0) continue;
        WalkableRegion region;
        region.outerContour = ToVectorContour(outerContour);
        for (const PathD& contour : walkableContours) {
            if (Area(contour) >= 0.0 || contour.empty()) continue;
            if (PointInPolygon(contour.front(), outerContour) == PointInPolygonResult::IsInside)
                region.holeContours.push_back(ToVectorContour(contour));
        }
        regions.push_back(std::move(region));
    }
    return true;
}

} // namespace cdt::detail
