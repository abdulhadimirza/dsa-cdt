#include "raylib.h"
#include "imgui.h"
#include "rlImGui.h"

#include "app/CameraController.hpp"
#include "app/NavmeshBakeCoordinator.hpp"
#include "app/NavmeshBakeSession.hpp"
#include "app/NavmeshInputKey.hpp"
#include "cdt/ConstrainedDelaunay.hpp"
#include "editor/EditorController.hpp"
#include "model/GeometryModel.hpp"
#include "render/SceneRenderer.hpp"
#include "validation/GeometryValidator.hpp"
#include "ui/ControlPanel.hpp"

#include <algorithm>
#include <iostream>
#include <optional>
#include <string>

namespace {

constexpr int screen_width = 1280;
constexpr int screen_height = 800;
constexpr Color background_color{22, 25, 32, 255};

} // namespace

int main() {
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(screen_width, screen_height, "DCDT Simulation - Infinite Canvas");
    SetExitKey(KEY_NULL);
    SetTargetFPS(60);
    rlImGuiSetup(true);

    cdt::CameraController camera(Vector2{0.0f, 0.0f});
    cdt::GeometryModel geometry;
    cdt::EditorController editor;
    cdt::SceneRenderer renderer;
    cdt::GeometryValidator validator;
    cdt::NavmeshBakeCoordinator bake_coordinator;
    cdt::NavmeshBakeSession bake_session;
    cdt::ControlPanel control_panel;
    double agent_radius = 0.0;

    while (!WindowShouldClose()) {
        // Update navigation and edit input before taking one validation snapshot for this frame.
        const Vector2 mouse_screen = GetMousePosition();
        const bool mouse_captured = ImGui::GetIO().WantCaptureMouse;
        if (!mouse_captured) camera.Update();

        const cdt::Vector2d mouse_world = camera.ScreenToWorld(mouse_screen);
        const bool panning = camera.IsPanning() || IsKeyDown(KEY_SPACE);
        cdt::EditorInput editor_input{};
        editor_input.cursor_world = mouse_world;
        editor_input.pick_radius_world = std::max(8.5, 8.0 / static_cast<double>(camera.GetZoom()));
        editor_input.editing_allowed = !mouse_captured && !panning;
        editor_input.left_pressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
        editor_input.left_down = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
        editor_input.left_released = IsMouseButtonReleased(MOUSE_BUTTON_LEFT);
        editor_input.right_pressed = IsMouseButtonPressed(MOUSE_BUTTON_RIGHT);
        editor_input.escape_pressed = IsKeyPressed(KEY_ESCAPE);
        editor.Update(editor_input, geometry);
        const cdt::GeometryValidationReport validation = validator.Validate(geometry);
        const std::string input_key = cdt::MakeNavmeshInputKey(validation.validated_navmesh_input, agent_radius);

        if (auto completion = bake_coordinator.TakeCompletion()) {
            if (bake_session.Apply(std::move(*completion)) &&
                bake_session.last_bake_status == cdt::GenerationStatus::Failed)
                std::cerr << "Navmesh generation failed: " << completion->result.message << '\n';
        }

        // Rendering uses the accepted mesh while validation report drives live loop diagnostics.
        BeginDrawing();
        ClearBackground(background_color);
        BeginMode2D(camera.GetCamera());
        camera.DrawWorldGrid();
        renderer.DrawWorld(geometry, validation, editor.State(),
            control_panel.Options(), bake_session.baked_mesh.has_value() ? &*bake_session.baked_mesh : nullptr);
        EndMode2D();

        renderer.DrawOverlays(geometry, control_panel.Options(), camera.GetCamera(), editor.State());
        rlImGuiBegin();
        const cdt::ControlRequests requests = control_panel.Draw(
            camera.GetCamera(), mouse_screen, mouse_world, geometry, editor.State(),
            bake_session.baked_mesh.has_value(), bake_session.bake_attempted, bake_session.last_bake_status,
            bake_session.baked_mesh.has_value() && input_key != bake_session.baked_input_key, bake_coordinator.IsRunning());
        if (requests.agent_radius) agent_radius = *requests.agent_radius;
        // Recompute after UI input so a radius edit changes the request key immediately.
        const std::string bake_input_key = cdt::MakeNavmeshInputKey(validation.validated_navmesh_input, agent_radius);
        rlImGuiEnd();

        // Apply UI requests after drawing; manual and automatic requests share the same snapshot/key.
        if (requests.reset_view) camera.ResetView(Vector2{0.0f, 0.0f});
        if (requests.selected_tool) editor.SetTool(*requests.selected_tool);
        if (requests.selected_edit_behavior) editor.SetEditBehavior(*requests.selected_edit_behavior);
        if (requests.bake_navmesh) {
            bake_session.bake_attempted = true;
            (void)bake_coordinator.Request(validation.validated_navmesh_input, agent_radius, bake_input_key);
        }
        if (requests.delete_navmesh) bake_session.baked_mesh.reset();
        if (requests.clear_geometry) {
            geometry.Clear();
            editor.ClearInteraction();
        }

        if (control_panel.AutoBakeNavmesh() && bake_session.baked_mesh && bake_input_key != bake_session.baked_input_key &&
            bake_input_key != bake_session.last_auto_requested_key) {
            (void)bake_coordinator.Request(validation.validated_navmesh_input, agent_radius, bake_input_key);
            bake_session.last_auto_requested_key = bake_input_key;
            bake_session.bake_attempted = true;
        }

        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
