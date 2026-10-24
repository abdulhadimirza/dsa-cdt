#pragma once

#include "model/GeometryModel.hpp"

#include <optional>
#include <utility>
#include <vector>

namespace cdt {

/// Per-loop diagnostic flags; the report keeps this vector parallel to GeometryModel::Loops().
struct LoopValidation {
    /// Position of the corresponding GeometryModel loop.
    std::size_t loop_index{};
    /// The loop independently passes finite-coordinate, area, edge, and self-crossing checks.
    bool individually_valid{false};
    /// This is the selected active border.
    bool active_border{false};
    /// This loop can participate in the generated walkable geometry.
    bool eligible{false};
    /// For an obstacle, every vertex lies strictly inside the active border.
    bool inside_active_border{false};
    /// The loop intersects the active border or another relevant obstacle.
    bool intersects_conflict{false};
};

/// One source loop copied into the navmesh input snapshot after validation.
struct ValidatedLoop {
    /// Index of the originating model loop, retained for diagnostics and traceability.
    std::size_t source_loop_index{};
    /// Original vertex IDs align position-for-position with vertices.
    std::vector<VertexId> vertex_ids;
    /// Finite positions copied from the model, so downstream stages need no model lookups.
    std::vector<Vector2d> vertices;
};

/// Immutable navmesh-ready snapshot; only GeometryValidator can create one from model data.
class ValidatedNavmeshInput {
public:
    /// Return the validated active border.
    [[nodiscard]] const ValidatedLoop& ActiveBorder() const noexcept { return active_border_; }
    /// Return obstacles classified as eligible and non-conflicting by validation.
    [[nodiscard]] const std::vector<ValidatedLoop>& EligibleObstacles() const noexcept { return obstacles_; }
private:
    friend class GeometryValidator;
    ValidatedNavmeshInput(ValidatedLoop border, std::vector<ValidatedLoop> obstacles)
        : active_border_(std::move(border)), obstacles_(std::move(obstacles)) {}
    ValidatedLoop active_border_;
    std::vector<ValidatedLoop> obstacles_;
};

/// Complete per-loop validation diagnostics and, when available, a navmesh-ready input snapshot.
struct GeometryValidationReport {
    /// Active border's model-loop index, if any individually valid border exists.
    std::optional<std::size_t> active_border_index;
    /// One diagnostic record per model loop, preserving model-loop order.
    std::vector<LoopValidation> loops;
    /// Present only when validation selected a border and copied its usable inputs.
    std::optional<ValidatedNavmeshInput> validated_navmesh_input;
};

class GeometryValidator {
public:
    /// Validate loops, select the active border, classify obstacles, and snapshot eligible inputs.
    [[nodiscard]] GeometryValidationReport Validate(const GeometryModel& model) const;
};

} // namespace cdt
