#pragma once
#include "cdt/ConstrainedDelaunay.hpp"
#include "validation/GeometryValidator.hpp"
#include <optional>

namespace cdt {

/// Coordinates offset-region construction and triangulation and maps user-visible statuses.
class NavmeshPipeline {
public:
    /// Orchestrate offset-region creation and triangulation, preserving user-visible status meanings.
    [[nodiscard]] GenerationResult Generate(
        const std::optional<ValidatedNavmeshInput>& geometry, double agentRadius, bool forceFull = false);
    /// Discard incremental history after a user clear or map replacement.
    void Reset() noexcept;

private:
    /// Triangulator retains mutable topology between worker requests.
    ConstrainedDelaunay triangulator;
    /// Last successfully published walkable contours are the baseline for geometry deltas.
    std::optional<detail::WalkableRegionSet> previousRegions;
    /// Border and radius identify changes that require a global rebuild.
    std::optional<ValidatedLoop> previousActiveBorder;
    double previousAgentRadius{};
};
}
