#pragma once
#include "core/Vector2d.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace cdt::detail {

/// One connected walkable area and its excluded holes, with source identity where unchanged.
struct WalkableRegion {
    /// Sanitized outer boundary in the winding convention used by Clipper.
    std::vector<Vector2d> outerContour;
    /// Optional original vertex IDs aligned one-for-one with outerContour.
    std::vector<std::optional<std::uint64_t>> outerSourceIds;
    /// Sanitized hole boundaries assigned to this outer contour.
    std::vector<std::vector<Vector2d>> holeContours;
    /// Optional IDs aligned with each corresponding holeContours entry and vertex.
    std::vector<std::vector<std::optional<std::uint64_t>>> holeSourceIds;
};

/// Non-empty, builder-created set of sanitized offset contours ready for triangulation.
class WalkableRegionSet {
public:
    [[nodiscard]] const std::vector<WalkableRegion>& Regions() const noexcept { return regions_; }
private:
    friend struct WalkableRegionSetBuilder;
    explicit WalkableRegionSet(std::vector<WalkableRegion> regions) : regions_(std::move(regions)) {}
    std::vector<WalkableRegion> regions_;
};

/// Walkable space added to or removed from the prior validated region set.
struct WalkableRegionDelta {
    /// Areas that became walkable in the new input, such as a shrinking obstacle.
    std::vector<WalkableRegion> added;
    /// Areas that stopped being walkable in the new input, such as a growing obstacle.
    std::vector<WalkableRegion> removed;
    /// Return true when both snapshots describe the same walkable area.
    [[nodiscard]] bool Empty() const noexcept { return added.empty() && removed.empty(); }
};

}
