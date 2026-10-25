#include "navmesh/NavmeshPipeline.hpp"
#include "navmesh/WalkableRegionBuilder.hpp"
#include <utility>

namespace cdt {

GenerationResult NavmeshPipeline::Generate(
    const std::optional<ValidatedNavmeshInput>& geometry, double agentRadius, bool forceFull) {
    // Validation owns source-geometry diagnostics; no snapshot means no active border was approved.
    if (!geometry) {
        Reset();
        return {GenerationStatus::NoActiveBorder, {}, "No valid active border."};
    }
    std::string error;
    auto regions = detail::BuildWalkableRegions(*geometry, agentRadius, error);
    // Offset failures are generation failures, while valid regions proceed to the CDT stage.
    if (!regions) return {GenerationStatus::Failed, {}, std::move(error)};

    const ValidatedLoop& currentBorder = geometry->ActiveBorder();
    const bool sameBorder = previousActiveBorder &&
        previousActiveBorder->vertex_ids == currentBorder.vertex_ids &&
        previousActiveBorder->vertices == currentBorder.vertices;
    const bool canPatch = !forceFull && previousRegions && sameBorder && previousAgentRadius == agentRadius;
    GenerationResult result;
    if (canPatch) {
        auto delta = detail::DiffWalkableRegions(*previousRegions, *regions, error);
        if (delta) result = triangulator.Update(*regions, *delta);
        if (!delta || result.status != GenerationStatus::Success)
            result = triangulator.Triangulate(*regions);
    } else {
        result = triangulator.Triangulate(*regions);
    }
    if (result.status == GenerationStatus::Success) {
        previousRegions = std::move(*regions);
        previousActiveBorder = currentBorder;
        previousAgentRadius = agentRadius;
    }
    return result;
}

void NavmeshPipeline::Reset() noexcept {
    triangulator.Reset();
    previousRegions.reset();
    previousActiveBorder.reset();
    previousAgentRadius = 0.0;
}
}
