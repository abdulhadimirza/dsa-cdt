#pragma once

#include "editor/EditorState.hpp"
#include "model/GeometryModel.hpp"

#include <optional>
#include <string>

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

/// Describes an accepted canvas click without interpreting the selected tool's behavior.
struct CanvasClick {
    EditorTool tool;
    Vector2d world_position;
};

class EditorController {
public:
    /// Applies one frame of editor input and reports an accepted generic click for app-level dispatch.
    /// @return The selected tool and world position when editing is allowed and the left button was pressed.
    [[nodiscard]] std::optional<CanvasClick> Update(const EditorInput& input, GeometryModel& model);
    /// Sets transient feedback displayed by editor overlays and the control panel.
    void SetStatusMessage(std::string message);
    /// Selects the active canvas tool and clears any in-progress interaction.
    void SetTool(EditorTool tool) noexcept;
    /// Changes vertex movement behavior and cancels any drag in progress.
    void SetEditBehavior(EditBehavior behavior) noexcept;
    /// Clears hover, drag, chain-drawing, and status state.
    void ClearInteraction() noexcept;
    /// Returns the current editor state for rendering and controls.
    [[nodiscard]] const EditorState& State() const noexcept { return state; }

private:
    /// Finds the closest vertex within the supplied world-space picking radius.
    [[nodiscard]] static std::optional<VertexId> PickVertex(
        const GeometryModel& model, Vector2d cursor, double radius) noexcept;
    EditorState state{};
};

} // namespace cdt
