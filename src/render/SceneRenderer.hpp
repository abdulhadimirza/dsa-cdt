#pragma once

#include "editor/EditorState.hpp"
#include "cdt/ConstrainedDelaunay.hpp"
#include "model/GeometryModel.hpp"
#include "pathfinding/PathfindingView.hpp"
#include "validation/GeometryValidator.hpp"
#include "raylib.h"

namespace cdt {

struct RenderOptions {
    bool show_coordinates{false};
    float navmesh_edge_transparency{50.0f};
};

class SceneRenderer {
public:
    /// Draws geometry, diagnostics, the accepted mesh, and a read-only pathfinding view in world space.
    void DrawWorld(const GeometryModel& model, const GeometryValidationReport& validation, const EditorState& editor,
        const RenderOptions& options, const NavMesh* mesh,
        const pathfinding::PathfindingView& pathfinding) const;
    /// Draw screen-space labels and editor status overlays.
    void DrawOverlays(const GeometryModel& model, const RenderOptions& options,
        const Camera2D& camera, const EditorState& editor) const;
};

} // namespace cdt
