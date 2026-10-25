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
        const std::optional<ValidatedNavmeshInput>& geometry, double agentRadius) const;
};
}
