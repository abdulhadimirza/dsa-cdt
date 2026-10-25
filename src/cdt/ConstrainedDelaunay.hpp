#pragma once

#include "navmesh/WalkableRegion.hpp"
#include "navmesh/NavMesh.hpp"

#include <memory>

namespace cdt {

/// Triangulates already-offset and validated walkable regions.
class ConstrainedDelaunay {
public:
    /// Construct an empty triangulator with no reusable mesh state.
    ConstrainedDelaunay();
    /// Release any retained mutable triangulation state.
    ~ConstrainedDelaunay();
    ConstrainedDelaunay(const ConstrainedDelaunay&) = delete;
    ConstrainedDelaunay& operator=(const ConstrainedDelaunay&) = delete;
    ConstrainedDelaunay(ConstrainedDelaunay&&) noexcept;
    ConstrainedDelaunay& operator=(ConstrainedDelaunay&&) noexcept;

    /// Triangulate sanitized walkable contours. Input validation and offsetting belong upstream.
    [[nodiscard]] GenerationResult Triangulate(const detail::WalkableRegionSet& regions);
    /// Replace the local mesh cavity for a walkability delta; failure leaves retained state intact.
    [[nodiscard]] GenerationResult Update(const detail::WalkableRegionSet& currentRegions,
        const detail::WalkableRegionDelta& delta);
    /// Discard reusable topology after a global input change or user clear.
    void Reset() noexcept;

private:
    struct State;
    /// Assemble an immutable output snapshot from the currently valid mutable topology.
    [[nodiscard]] std::optional<NavMesh> CreateSnapshot() const;
    std::unique_ptr<State> state_;
};

} // namespace cdt
