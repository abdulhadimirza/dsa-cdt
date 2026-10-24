#pragma once

#include "navmesh/WalkableRegion.hpp"
#include "navmesh/NavMesh.hpp"

namespace cdt {

/// Triangulates already-offset and validated walkable regions.
class ConstrainedDelaunay {
public:
    /// Triangulate sanitized walkable contours. Input validation and offsetting belong upstream.
    [[nodiscard]] GenerationResult Triangulate(const detail::WalkableRegionSet& regions) const;
};

} // namespace cdt
