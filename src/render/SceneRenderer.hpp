#pragma once

#include "editor/EditorState.hpp"
#include "cdt/ConstrainedDelaunay.hpp"
#include "model/GeometryModel.hpp"
#include "validation/GeometryValidator.hpp"
#include "pathfinding/Pathfinder.hpp"
#include "raylib.h"

namespace cdt {

struct RenderOptions {
    bool show_coordinates{false};
    float navmesh_edge_transparency{80.0f};
};

class SceneRenderer {
public:
    /// Draw the editable model with validation diagnostics and an optional immutable baked mesh.
    void DrawWorld(const GeometryModel& model, const GeometryValidationReport& validation, const EditorState& editor,
        const RenderOptions& options, const NavMesh* mesh,
        const std::optional<pathfinding::PathResult>& path) const;
    /// Draw screen-space labels and editor status overlays.
    void DrawOverlays(const GeometryModel& model, const RenderOptions& options,
        const Camera2D& camera, const EditorState& editor) const;
};

} // namespace cdt
