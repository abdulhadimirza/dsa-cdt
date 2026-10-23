#pragma once

#include "core/Vector2d.hpp"
#include "cdt/ConstrainedDelaunay.hpp"
#include "editor/EditorState.hpp"
#include "model/GeometryModel.hpp"
#include "render/SceneRenderer.hpp"

#include <optional>

#include "raylib.h"

namespace cdt {

struct ControlRequests {
    bool reset_view{false};
    bool clear_geometry{false};
    bool bake_navmesh{false};
    bool delete_navmesh{false};
    std::optional<EditorTool> selected_tool;
    std::optional<EditBehavior> selected_edit_behavior;
};

class ControlPanel {
public:
    [[nodiscard]] ControlRequests Draw(const Camera2D& camera, Vector2 mouse_screen,
        Vector2d mouse_world, const GeometryModel& model, const EditorState& editor,
        bool has_baked_mesh, bool bake_attempted, GenerationStatus last_bake_status, bool mesh_out_of_date, bool bake_running);
    [[nodiscard]] const RenderOptions& Options() const noexcept { return options; }
    [[nodiscard]] bool AutoBakeNavmesh() const noexcept { return auto_bake_navmesh; }

private:
    RenderOptions options{};
    bool auto_bake_navmesh{false};
};

} // namespace cdt
