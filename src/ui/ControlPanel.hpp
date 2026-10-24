#pragma once

#include "core/Vector2d.hpp"
#include "cdt/ConstrainedDelaunay.hpp"
#include "editor/EditorState.hpp"
#include "model/GeometryModel.hpp"
#include "pathfinding/PathfindingView.hpp"
#include "render/SceneRenderer.hpp"

#include <optional>

#include "raylib.h"

namespace cdt {

struct ControlRequests {
    bool reset_view{false};
    bool clear_geometry{false};
    bool bake_navmesh{false};
    bool delete_navmesh{false};
    bool generate_path{false};
    std::optional<EditorTool> selected_tool;
    std::optional<EditBehavior> selected_edit_behavior;
    std::optional<double> agent_radius;
};

/// Bake status displayed by the control panel for the currently accepted mesh.
struct BakePanelView {
    bool has_baked_mesh{};
    bool attempted{};
    GenerationStatus last_status{GenerationStatus::Failed};
    bool out_of_date{};
    bool running{};
};

/// Read-only inputs required to draw one control-panel frame, grouped by concern.
struct ControlPanelContext {
    Camera2D camera;
    Vector2 mouse_screen;
    Vector2d mouse_world;
    const GeometryModel& model;
    const EditorState& editor;
    BakePanelView bake;
    pathfinding::PathfindingView pathfinding;
};

class ControlPanel {
public:
    /// Draws the panel from a frame context and returns actions for the app to apply.
    [[nodiscard]] ControlRequests Draw(const ControlPanelContext& context);
    /// Returns the current renderer options selected in the panel.
    [[nodiscard]] const RenderOptions& Options() const noexcept { return options; }
    /// Reports whether the panel's automatic navmesh baking option is enabled.
    [[nodiscard]] bool AutoBakeNavmesh() const noexcept { return auto_bake_navmesh; }
    /// Reports whether the panel's automatic route update option is enabled.
    [[nodiscard]] bool AutoUpdatePath() const noexcept { return auto_update_path; }

private:
    /// Draws endpoint, route status, and path generation controls.
    void DrawPathfindingControls(ControlRequests& requests, const pathfinding::PathfindingView& pathfinding,
        bool has_baked_mesh);

    RenderOptions options{};
    bool auto_bake_navmesh{false};
    bool auto_update_path{false};
    double agent_radius{0.0};
};

} // namespace cdt
