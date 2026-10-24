#include "navmesh/NavmeshPipeline.hpp"
#include "navmesh/WalkableRegionBuilder.hpp"
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string_view>
#include <utility>

namespace cdt {
namespace {

using DiagnosticClock = std::chrono::steady_clock;

bool NavmeshDiagnosticsEnabled() {
    const char* value = std::getenv("CDT_NAVMESH_DIAGNOSTICS");
    return value && value[0] != '\0' && std::string_view(value) != "0";
}

double ElapsedMilliseconds(DiagnosticClock::time_point start) {
    return std::chrono::duration<double, std::milli>(DiagnosticClock::now() - start).count();
}

const char* StatusName(GenerationStatus status) {
    switch (status) {
    case GenerationStatus::Success: return "success";
    case GenerationStatus::NoActiveBorder: return "no-active-border";
    case GenerationStatus::Failed: return "failed";
    }
    return "unknown";
}

}

GenerationResult NavmeshPipeline::Generate(
    const std::optional<ValidatedNavmeshInput>& geometry, double agentRadius) const {
    const bool diagnosticsEnabled = NavmeshDiagnosticsEnabled();
    const auto started = diagnosticsEnabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    // Validation owns source-geometry diagnostics; no snapshot means no active border was approved.
    if (!geometry) {
        if (diagnosticsEnabled)
            std::clog << "[navmesh-pipeline] result=no-active-border total_ms="
                << ElapsedMilliseconds(started) << '\n';
        return {GenerationStatus::NoActiveBorder, {}, "No valid active border."};
    }

    std::size_t sourceObstacleVertices = 0;
    if (diagnosticsEnabled)
        for (const auto& obstacle : geometry->EligibleObstacles())
            sourceObstacleVertices += obstacle.vertices.size();
    std::string error;
    const auto regionBuildStarted = diagnosticsEnabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    auto regions = detail::BuildWalkableRegions(*geometry, agentRadius, error);
    const double regionBuildMs = diagnosticsEnabled ? ElapsedMilliseconds(regionBuildStarted) : 0.0;
    // Offset failures are generation failures, while valid regions proceed to the CDT stage.
    if (!regions) {
        if (diagnosticsEnabled)
            std::clog << std::fixed << std::setprecision(3)
                << "[navmesh-pipeline] result=failed stage=walkable-region-build"
                << " radius=" << agentRadius
                << " border_vertices=" << geometry->ActiveBorder().vertices.size()
                << " obstacles=" << geometry->EligibleObstacles().size()
                << " obstacle_vertices=" << sourceObstacleVertices
                << " region_build_ms=" << regionBuildMs
                << " total_ms=" << ElapsedMilliseconds(started)
                << " message=" << std::quoted(error) << '\n';
        return {GenerationStatus::Failed, {}, std::move(error)};
    }

    std::size_t walkableContours = 0;
    std::size_t walkableVertices = 0;
    if (diagnosticsEnabled)
        for (const auto& region : regions->Regions()) {
            ++walkableContours;
            walkableVertices += region.outerContour.size();
            walkableContours += region.holeContours.size();
            for (const auto& hole : region.holeContours) walkableVertices += hole.size();
        }
    const auto triangulationStarted = diagnosticsEnabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    GenerationResult result = ConstrainedDelaunay{}.Triangulate(*regions);
    const double triangulationMs = diagnosticsEnabled ? ElapsedMilliseconds(triangulationStarted) : 0.0;
    if (diagnosticsEnabled) {
        const std::size_t generatedVertices = result.mesh ? result.mesh->Vertices().size() : 0;
        const std::size_t generatedTriangles = result.mesh ? result.mesh->Triangles().size() : 0;
        std::clog << std::fixed << std::setprecision(3)
            << "[navmesh-pipeline] result=" << StatusName(result.status)
            << " radius=" << agentRadius
            << " border_vertices=" << geometry->ActiveBorder().vertices.size()
            << " obstacles=" << geometry->EligibleObstacles().size()
            << " obstacle_vertices=" << sourceObstacleVertices
            << " walkable_regions=" << regions->Regions().size()
            << " walkable_contours=" << walkableContours
            << " walkable_vertices=" << walkableVertices
            << " region_build_ms=" << regionBuildMs
            << " triangulation_ms=" << triangulationMs
            << " total_ms=" << ElapsedMilliseconds(started)
            << " output_vertices=" << generatedVertices
            << " output_triangles=" << generatedTriangles
            << " message=" << std::quoted(result.message) << '\n';
    }
    return result;
}
}
