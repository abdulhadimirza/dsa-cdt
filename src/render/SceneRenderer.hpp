#pragma once

#include "editor/EditorState.hpp"
#include "model/GeometryModel.hpp"
#include "raylib.h"

namespace cdt {

struct RenderOptions {
    bool show_vertex_ids{false};
    bool show_coordinates{false};
};

class SceneRenderer {
public:
    void DrawWorld(const GeometryModel& model, const EditorState& editor) const;
    void DrawOverlays(const GeometryModel& model, const RenderOptions& options,
        const Camera2D& camera, const EditorState& editor) const;
};

} // namespace cdt
