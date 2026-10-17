#include "raylib.h"
#include "imgui.h"
#include "rlImGui.h"

#include "app/CameraController.hpp"
#include "editor/EditorController.hpp"
#include "model/GeometryModel.hpp"
#include "render/SceneRenderer.hpp"
#include "validation/GeometryValidator.hpp"
#include "ui/ControlPanel.hpp"

#include <algorithm>

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
    cdt::ControlPanel control_panel;

    while (!WindowShouldClose()) {
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
        const cdt::GeometryValidation validation = validator.Validate(geometry);

        BeginDrawing();
        ClearBackground(background_color);
        BeginMode2D(camera.GetCamera());
        camera.DrawWorldGrid();
        renderer.DrawWorld(geometry, validation, editor.State());
        EndMode2D();

        renderer.DrawOverlays(geometry, control_panel.Options(), camera.GetCamera(), editor.State());
        rlImGuiBegin();
        const cdt::ControlRequests requests = control_panel.Draw(
            camera.GetCamera(), mouse_screen, mouse_world, geometry, editor.State());
        rlImGuiEnd();

        if (requests.reset_view) camera.ResetView(Vector2{0.0f, 0.0f});
        if (requests.selected_tool) editor.SetTool(*requests.selected_tool);
        if (requests.selected_edit_behavior) editor.SetEditBehavior(*requests.selected_edit_behavior);
        if (requests.clear_geometry) {
            geometry.Clear();
            editor.ClearInteraction();
        }

        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
