#include "raylib.h"
#include "imgui.h"
#include "rlImGui.h"

#include "app/CameraController.hpp"
#include "editor/EditorController.hpp"
#include "model/GeometryModel.hpp"
#include "render/SceneRenderer.hpp"
#include "ui/ControlPanel.hpp"

#include <algorithm>
#include <array>
#include <vector>

namespace {

constexpr int screen_width = 1280;
constexpr int screen_height = 800;
constexpr Color background_color{22, 25, 32, 255};

constexpr std::array<cdt::Vector2d, 10> default_vertices{{
    {-380.0, -180.0}, {-180.0, -240.0}, {80.0, -220.0}, {320.0, -150.0},
    {420.0, 80.0}, {220.0, 240.0}, {-100.0, 220.0}, {-330.0, 120.0},
    {-150.0, -30.0}, {90.0, 10.0}
}};

} // namespace

int main() {
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(screen_width, screen_height, "DCDT Simulation - Infinite Canvas");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    cdt::CameraController camera(Vector2{0.0f, 0.0f});
    cdt::GeometryModel geometry;
    geometry.ResetVertices(std::vector<cdt::Vector2d>(default_vertices.begin(), default_vertices.end()));
    cdt::EditorController editor;
    cdt::SceneRenderer renderer;
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
        editor.Update(editor_input, geometry);

        BeginDrawing();
        ClearBackground(background_color);
        BeginMode2D(camera.GetCamera());
        camera.DrawWorldGrid();
        renderer.DrawWorld(geometry, editor.State());
        EndMode2D();

        renderer.DrawOverlays(geometry, control_panel.Options(), camera.GetCamera());
        rlImGuiBegin();
        const cdt::ControlRequests requests = control_panel.Draw(
            camera.GetCamera(), mouse_screen, mouse_world, geometry, editor.State());
        rlImGuiEnd();

        if (requests.reset_view) camera.ResetView(Vector2{0.0f, 0.0f});
        if (requests.reset_default_vertices) {
            geometry.ResetVertices(std::vector<cdt::Vector2d>(default_vertices.begin(), default_vertices.end()));
            editor.ClearInteraction();
        }
        if (requests.generate_random_vertices) {
            geometry.Clear();
            editor.ClearInteraction();
            for (int i = 0; i < 15; ++i) {
                const double x = static_cast<double>(GetRandomValue(-450, 450));
                const double y = static_cast<double>(GetRandomValue(-300, 300));
                (void)geometry.AddVertex(cdt::Vector2d{x, y});
            }
        }
        if (requests.clear_vertices) {
            geometry.Clear();
            editor.ClearInteraction();
        }

        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
