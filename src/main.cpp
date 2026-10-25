#include "raylib.h"
#include "imgui.h"
#include "rlImGui.h"

#include "app/CameraController.hpp"
#include "app/GeometryDerivedDataCache.hpp"
#include "app/NavmeshBakeCoordinator.hpp"
#include "app/NavmeshBakeSession.hpp"
#include "app/MapFile.hpp"
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

/// Run each editor frame: apply input, reuse or refresh geometry-derived data, handle UI requests,
/// submit bake work, and render with the same current geometry report and bake signature. Panel
/// load and clear actions are applied before rendering so their geometry appears in that frame.
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
    // Keep the validation report and bake signature together so all frame consumers see matching inputs.
    cdt::GeometryDerivedDataCache geometry_derived_data;

    while (!WindowShouldClose()) {
        // Update camera and editor input before refreshing the report for this frame.
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
        // Apply edits before refreshing so the panel, bake checks, and drawing share the new geometry.
        const auto canvas_click = editor.Update(editor_input, geometry);
        geometry_derived_data.Refresh(geometry, validator, agent_radius);

        // Track whether the newly completed bake replaced the mesh used by navigation this frame.
        bool mesh_replaced = false;
        if (auto completion = bake_coordinator.TakeCompletion()) {
            const bool applied = bake_session.Apply(std::move(*completion));
            mesh_replaced = applied && bake_session.last_bake_status == cdt::GenerationStatus::Success;
            if (applied && bake_session.last_bake_status == cdt::GenerationStatus::Failed)
                std::cerr << "Navmesh generation failed: " << completion->result.message << '\n';
        }

        // Update navigation to the accepted mesh before handling any endpoint click this frame.
        if (mesh_replaced && bake_session.baked_mesh)
            pathfinding_session.OnMeshAccepted(*bake_session.baked_mesh, control_panel.AutoUpdatePath());
        // Endpoint clicks must use the currently accepted mesh; a new bake may have changed it above.
        const cdt::NavMesh* accepted_mesh = bake_session.baked_mesh ? &*bake_session.baked_mesh : nullptr;
        if (canvas_click)
            DispatchNavigationClick(*canvas_click, pathfinding_session, accepted_mesh,
                control_panel.AutoUpdatePath() && !mesh_replaced);

        // Collect panel actions before world rendering so load/clear affects this frame's geometry.
        BeginDrawing();
        ClearBackground(background_color);
        rlImGuiBegin();
        // Capture the route state shown in this frame's control panel.
        const cdt::pathfinding::PathfindingView panel_pathfinding_view = pathfinding_session.View();
        // Give the panel the current model, editor state, bake status, and route details to display.
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
                .out_of_date = bake_session.baked_mesh.has_value() &&
                    geometry_derived_data.BakeInputSignature() != bake_session.baked_input_signature,
                .running = bake_coordinator.IsRunning(),
            },
            .pathfinding = panel_pathfinding_view,
        };
        // Gather button and field changes first; geometry requests are applied before world drawing below.
        const cdt::ControlRequests requests = control_panel.Draw(panel_context);
        // Keep a newly entered radius for the cache refresh and any bake request later this frame.
        if (requests.agent_radius) agent_radius = *requests.agent_radius;

        // Apply UI requests before rendering; successful load and clear invalidate the report and signature immediately.
        if (requests.reset_view) camera.ResetView(Vector2{0.0f, 0.0f});
        if (requests.selected_tool) editor.SetTool(*requests.selected_tool);
        if (requests.selected_edit_behavior) editor.SetEditBehavior(*requests.selected_edit_behavior);
        if (requests.clear_path) pathfinding_session.ClearPath();
        if (requests.clear_endpoints) pathfinding_session.Clear();
        // Enabling auto-update computes the initial route once; later changes are handled by SetEndpoint/OnMeshAccepted.
        if (requests.auto_update_path_enabled && bake_session.baked_mesh)
            pathfinding_session.Generate(*bake_session.baked_mesh);
        if (requests.save_map) {
            std::string error;
            if (cdt::SaveMapFile(requests.map_path, geometry, error))
                control_panel.SetMapStatus("Map saved.");
            else control_panel.SetMapStatus(error);
        }
        // LoadMapFile replaces geometry only after the entire file is accepted, so a failed load
        // leaves both the visible map and the cache's geometry version unchanged.
        if (requests.load_map) {
            std::string error;
            if (cdt::LoadMapFile(requests.map_path, geometry, error)) {
                editor.ClearInteraction();
                pathfinding_session.Clear();
                bake_session.Reset(bake_coordinator.InvalidatePending());
                control_panel.SetMapStatus("Map loaded. Navmesh cleared.");
            } else {
                control_panel.SetMapStatus(error);
            }
        }
        // Clearing advances the geometry version; the refresh below then replaces the old report.
        if (requests.clear_geometry) {
            geometry.Clear();
            editor.ClearInteraction();
        }

        // Refresh after all UI geometry requests so rendering and baking use data for the final
        // geometry state of this frame.
        geometry_derived_data.Refresh(geometry, validator, agent_radius);
        // Manual bake requests use the just-refreshed report and signature.
        if (requests.bake_navmesh) {
            bake_session.bake_attempted = true;
            (void)bake_coordinator.Request(geometry_derived_data.ValidationReport().validated_navmesh_input,
                agent_radius, geometry_derived_data.BakeInputSignature());
            bake_session.last_requested_signature = geometry_derived_data.BakeInputSignature();
        }
        if (requests.clear_navmesh) {
            // Reject queued/running bake results, then mark this input handled so auto-bake does not undo the clear.
            bake_session.Reset(bake_coordinator.InvalidatePending());
            bake_session.last_requested_signature = geometry_derived_data.BakeInputSignature();
            pathfinding_session.ClearForMeshDeletion();
        }

        // Generate a route only when a baked mesh is available after the panel requests are applied.
        if (requests.generate_path && bake_session.baked_mesh)
            pathfinding_session.Generate(*bake_session.baked_mesh);

        // Capture the final route state after this frame's path requests for world drawing.
        const cdt::pathfinding::PathfindingView pathfinding_view = pathfinding_session.View();

        // Request automatic work once per changed signature, including the first bake when no mesh exists.
        if (control_panel.AutoBakeNavmesh() &&
            geometry_derived_data.BakeInputSignature() != bake_session.baked_input_signature &&
            geometry_derived_data.BakeInputSignature() != bake_session.last_requested_signature) {
            (void)bake_coordinator.Request(geometry_derived_data.ValidationReport().validated_navmesh_input,
                agent_radius, geometry_derived_data.BakeInputSignature());
            bake_session.last_requested_signature = geometry_derived_data.BakeInputSignature();
            bake_session.bake_attempted = true;
        }

        // Draw only after load, clear, validation, and bake decisions use this frame's final geometry.
        BeginMode2D(camera.GetCamera());
        camera.DrawWorldGrid();
        renderer.DrawWorld(geometry, geometry_derived_data.ValidationReport(), editor.State(),
            control_panel.Options(), bake_session.baked_mesh.has_value() ? &*bake_session.baked_mesh : nullptr,
            pathfinding_view);
        EndMode2D();
        renderer.DrawOverlays(geometry, control_panel.Options(), camera.GetCamera(), editor.State());
        rlImGuiEnd();

        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
