#pragma once

#include "editor/EditorState.hpp"
#include "model/GeometryModel.hpp"

namespace cdt {

struct EditorInput {
    Vector2d cursor_world{};
    double pick_radius_world{8.5};
    bool editing_allowed{true};
    bool left_pressed{false};
    bool left_down{false};
    bool left_released{false};
    bool right_pressed{false};
    bool escape_pressed{false};
};

class EditorController {
public:
    void Update(const EditorInput& input, GeometryModel& model);
    void SetTool(EditorTool tool) noexcept;
    void ClearInteraction() noexcept;
    [[nodiscard]] const EditorState& State() const noexcept { return state; }

private:
    [[nodiscard]] static std::optional<VertexId> PickVertex(
        const GeometryModel& model, Vector2d cursor, double radius) noexcept;
    EditorState state{};
};

} // namespace cdt
