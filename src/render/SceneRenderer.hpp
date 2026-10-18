#pragma once

#include "editor/EditorState.hpp"
#include "cdt/ConstrainedDelaunay.hpp"
#include "model/GeometryModel.hpp"
#include "validation/GeometryValidator.hpp"
#include "raylib.h"

namespace cdt {

struct RenderOptions {
    bool show_vertex_ids{false};
    bool show_coordinates{false};
};

class SceneRenderer {
public:
    void DrawWorld(const GeometryModel& model, const GeometryValidation& validation, const EditorState& editor,
        const ConstrainedMesh* mesh) const;
    void DrawOverlays(const GeometryModel& model, const RenderOptions& options,
        const Camera2D& camera, const EditorState& editor) const;
};

} // namespace cdt
