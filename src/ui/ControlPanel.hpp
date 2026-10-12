#pragma once

#include "core/Vector2d.hpp"
#include "editor/EditorState.hpp"
#include "model/GeometryModel.hpp"
#include "render/SceneRenderer.hpp"

#include <optional>

#include "raylib.h"

namespace cdt {

struct ControlRequests {
    bool reset_view{false};
    bool clear_geometry{false};
    std::optional<EditorTool> selected_tool;
};

class ControlPanel {
public:
    [[nodiscard]] ControlRequests Draw(const Camera2D& camera, Vector2 mouse_screen,
        Vector2d mouse_world, const GeometryModel& model, const EditorState& editor);
    [[nodiscard]] const RenderOptions& Options() const noexcept { return options; }

private:
    RenderOptions options{};
};

} // namespace cdt
