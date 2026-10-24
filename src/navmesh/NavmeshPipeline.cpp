#include "navmesh/NavmeshPipeline.hpp"
#include "navmesh/WalkableRegionBuilder.hpp"
#include <utility>

namespace cdt {

GenerationResult NavmeshPipeline::Generate(
    const std::optional<ValidatedNavmeshInput>& geometry, double agentRadius) const {
    // Validation owns source-geometry diagnostics; no snapshot means no active border was approved.
    if (!geometry) return {GenerationStatus::NoActiveBorder, {}, "No valid active border."};
    std::string error;
    auto regions = detail::BuildWalkableRegions(*geometry, agentRadius, error);
    // Offset failures are generation failures, while valid regions proceed to the CDT stage.
    if (!regions) return {GenerationStatus::Failed, {}, std::move(error)};
    return ConstrainedDelaunay{}.Triangulate(*regions);
}
}
