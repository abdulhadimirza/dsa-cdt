#include "raylib.h"
#include "imgui.h"
#include "rlImGui.h"

#include "app/CameraController.hpp"
#include "app/NavmeshBakeCoordinator.hpp"
#include "app/NavmeshBakeSession.hpp"
#include "app/MapFile.hpp"
#include "app/NavmeshInputKey.hpp"
#include "app/PathfindingSession.hpp"
#include "cdt/ConstrainedDelaunay.hpp"
#include "editor/EditorController.hpp"
#include "model/GeometryModel.hpp"
#include "render/SceneRenderer.hpp"
#include "validation/GeometryValidator.hpp"
#include "ui/ControlPanel.hpp"

#include <algorithm>
#include <iostream>
#include <string>

namespace {

constexpr int screen_width = 1280;
constexpr int screen_height = 800;
constexpr Color background_color{22, 25, 32, 255};

/// Applies one endpoint placement update to the pathfinding session.
void DispatchNavigationClick(const cdt::CanvasClick& click, cdt::PathfindingSession& pathfinding,
    const cdt::NavMesh* mesh, bool auto_update) {
    cdt::PathEndpoint endpoint = cdt::PathEndpoint::Start;
    switch (click.tool) {
        case cdt::EditorTool::Start:
            endpoint = cdt::PathEndpoint::Start;
            break;
        case cdt::EditorTool::Goal:
            endpoint = cdt::PathEndpoint::Goal;
            break;
        default:
            return;
    }
    pathfinding.SetEndpoint(endpoint, click.world_position, mesh, auto_update);
}

} // namespace

/// Runs the editor, asynchronous bake, navigation session, rendering, and UI request loop.
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
    cdt::PathfindingSession pathfinding_session;
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
        const auto canvas_click = editor.Update(editor_input, geometry);
        const cdt::GeometryValidationReport validation = validator.Validate(geometry);
        const std::string input_key = cdt::MakeNavmeshInputKey(validation.validated_navmesh_input, agent_radius);

        bool mesh_replaced = false;
        if (auto completion = bake_coordinator.TakeCompletion()) {
            const bool applied = bake_session.Apply(std::move(*completion));
            mesh_replaced = applied && bake_session.last_bake_status == cdt::GenerationStatus::Success;
            if (applied && bake_session.last_bake_status == cdt::GenerationStatus::Failed)
                std::cerr << "Navmesh generation failed: " << completion->result.message << '\n';
        }

        const cdt::NavMesh* accepted_mesh = bake_session.baked_mesh ? &*bake_session.baked_mesh : nullptr;
        if (canvas_click)
            DispatchNavigationClick(*canvas_click, pathfinding_session, accepted_mesh,
                control_panel.AutoUpdatePath() && !mesh_replaced);
        if (mesh_replaced && bake_session.baked_mesh)
            pathfinding_session.OnMeshAccepted(*bake_session.baked_mesh, control_panel.AutoUpdatePath());

        const cdt::pathfinding::PathfindingView pathfinding_view = pathfinding_session.View();

        // Rendering uses the accepted mesh while validation report drives live loop diagnostics.
        BeginDrawing();
        ClearBackground(background_color);
        BeginMode2D(camera.GetCamera());
        camera.DrawWorldGrid();
        renderer.DrawWorld(geometry, validation, editor.State(),
            control_panel.Options(), bake_session.baked_mesh.has_value() ? &*bake_session.baked_mesh : nullptr,
            pathfinding_view);
        EndMode2D();

        renderer.DrawOverlays(geometry, control_panel.Options(), camera.GetCamera(), editor.State());
        rlImGuiBegin();
        const cdt::ControlPanelContext panel_context{
            .camera = camera.GetCamera(),
            .mouse_screen = mouse_screen,
            .mouse_world = mouse_world,
            .model = geometry,
            .editor = editor.State(),
            .bake = {
                .has_baked_mesh = bake_session.baked_mesh.has_value(),
                .attempted = bake_session.bake_attempted,
                .last_status = bake_session.last_bake_status,
                .out_of_date = bake_session.baked_mesh.has_value() && input_key != bake_session.baked_input_key,
                .running = bake_coordinator.IsRunning(),
            },
            .pathfinding = pathfinding_view,
        };
        const cdt::ControlRequests requests = control_panel.Draw(panel_context);
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
        if (requests.delete_navmesh) {
            bake_session.baked_mesh.reset();
            pathfinding_session.ClearForMeshDeletion();
        }
        if (requests.clear_path) pathfinding_session.ClearPath();
        if (requests.save_map) {
            std::string error;
            if (cdt::SaveMapFile(requests.map_path, geometry, error))
                control_panel.SetMapStatus("Map saved.");
            else control_panel.SetMapStatus(error);
        }
        bool map_loaded = false;
        if (requests.load_map) {
            std::string error;
            if (cdt::LoadMapFile(requests.map_path, geometry, error)) {
                editor.ClearInteraction();
                pathfinding_session.Clear();
                bake_session.Reset(bake_coordinator.InvalidatePending());
                control_panel.SetMapStatus("Map loaded. Bake the navmesh to use pathfinding.");
                map_loaded = true;
            } else {
                control_panel.SetMapStatus(error);
            }
        }
        if (requests.clear_geometry) {
            geometry.Clear();
            editor.ClearInteraction();
        }

        if (requests.generate_path && bake_session.baked_mesh)
            pathfinding_session.Generate(*bake_session.baked_mesh);

        if (!map_loaded && control_panel.AutoBakeNavmesh() && bake_session.baked_mesh && bake_input_key != bake_session.baked_input_key &&
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
