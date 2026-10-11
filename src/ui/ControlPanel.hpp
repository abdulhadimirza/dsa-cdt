#pragma once

#include "core/Vector2d.hpp"
#include "editor/EditorState.hpp"
#include "model/GeometryModel.hpp"
#include "render/SceneRenderer.hpp"

#include "raylib.h"

namespace cdt {

struct ControlRequests {
    bool reset_view{false};
    bool reset_default_vertices{false};
    bool generate_random_vertices{false};
    bool clear_vertices{false};
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
