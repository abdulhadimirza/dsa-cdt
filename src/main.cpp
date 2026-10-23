#include "raylib.h"
#include "imgui.h"
#include "rlImGui.h"

#include "app/CameraController.hpp"
#include "cdt/ConstrainedDelaunay.hpp"
#include "editor/EditorController.hpp"
#include "model/GeometryModel.hpp"
#include "render/SceneRenderer.hpp"
#include "validation/GeometryValidator.hpp"
#include "ui/ControlPanel.hpp"

#include <algorithm>
#include <iostream>
#include <optional>
#include <iomanip>
#include <sstream>
#include <string>

namespace {

constexpr int screen_width = 1280;
constexpr int screen_height = 800;
constexpr Color background_color{22, 25, 32, 255};

std::string TriangulationInputKey(const cdt::GeometryModel& model, const cdt::GeometryValidation& validation) {
    std::ostringstream key;
    key << std::setprecision(17);
    if (!validation.active_border_index) return "no-active-border";
    const auto append_loop = [&](std::size_t i) {
        key << 'L' << i << ':';
        for (cdt::VertexId id : model.Loops()[i].vertices) {
            const cdt::Vertex* v = model.FindVertex(id);
            key << id << ',';
            if (v) key << v->position.x << ',' << v->position.y;
            key << ';';
        }
    };
    append_loop(*validation.active_border_index);
    for (std::size_t i = 0; i < model.Loops().size() && i < validation.loops.size(); ++i)
        if (i != *validation.active_border_index && validation.loops[i].eligible &&
            model.Loops()[i].type == cdt::EdgeType::Obstacle) append_loop(i);
    return key.str();
}

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
    cdt::ConstrainedDelaunay triangulator;
    std::optional<cdt::ConstrainedMesh> baked_mesh;
    bool bake_attempted = false;
    cdt::GenerationStatus last_bake_status = cdt::GenerationStatus::Failed;
    std::string baked_input_key;
    std::string pending_input_key;
    double pending_since = 0.0;
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
        const std::string input_key = TriangulationInputKey(geometry, validation);

        BeginDrawing();
        ClearBackground(background_color);
        BeginMode2D(camera.GetCamera());
        camera.DrawWorldGrid();
        renderer.DrawWorld(geometry, validation, editor.State(), baked_mesh ? &*baked_mesh : nullptr);
        EndMode2D();

        renderer.DrawOverlays(geometry, control_panel.Options(), camera.GetCamera(), editor.State());
        rlImGuiBegin();
        const cdt::ControlRequests requests = control_panel.Draw(
            camera.GetCamera(), mouse_screen, mouse_world, geometry, editor.State(),
            baked_mesh.has_value(), bake_attempted, last_bake_status,
            baked_mesh.has_value() && input_key != baked_input_key);
        rlImGuiEnd();

        if (requests.reset_view) camera.ResetView(Vector2{0.0f, 0.0f});
        if (requests.selected_tool) editor.SetTool(*requests.selected_tool);
        if (requests.selected_edit_behavior) editor.SetEditBehavior(*requests.selected_edit_behavior);
        if (requests.bake_navmesh) {
            const cdt::GenerationResult result = triangulator.Generate(geometry, validation);
            bake_attempted = true;
            last_bake_status = result.status;
            if (result.status == cdt::GenerationStatus::Success) { baked_mesh = result.mesh; baked_input_key = input_key; }
            else if (result.status == cdt::GenerationStatus::Failed)
                std::cerr << "Navmesh generation failed: " << result.message << '\n';
        }
        if (requests.delete_navmesh) baked_mesh.reset();
        if (requests.clear_geometry) {
            geometry.Clear();
            editor.ClearInteraction();
        }

        if (control_panel.AutoBakeNavmesh() && baked_mesh && input_key != baked_input_key) {
            if (pending_input_key != input_key) { pending_input_key = input_key; pending_since = GetTime(); }
            else if (GetTime() - pending_since >= 0.25) {
                const cdt::GenerationResult result = triangulator.Generate(geometry, validation);
                bake_attempted = true;
                last_bake_status = result.status;
                if (result.status == cdt::GenerationStatus::Success) { baked_mesh = result.mesh; baked_input_key = input_key; }
                else if (result.status == cdt::GenerationStatus::Failed)
                    std::cerr << "Automatic navmesh generation failed: " << result.message << '\n';
                pending_input_key.clear();
            }
        } else pending_input_key.clear();

        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
