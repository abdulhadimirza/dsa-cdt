#include "ui/ControlPanel.hpp"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace cdt {
namespace {

const ImVec4 path_success_color{0.3f, 0.9f, 0.5f, 1.0f};
const ImVec4 path_failure_color{1.0f, 0.72f, 0.32f, 1.0f};
const ImVec4 path_header_color{0.3f, 0.8f, 1.0f, 1.0f};

} // namespace

/// Draws endpoint values, route status, and path-generation controls from read-only data.
void ControlPanel::DrawPathfindingControls(ControlRequests& requests,
    const pathfinding::PathfindingView& view, bool has_baked_mesh) {
    ImGui::TextColored(path_header_color, "Pathfinding");
    if (view.start)
        ImGui::Text("Start: (%.1f, %.1f)", view.start->x, view.start->y);
    else ImGui::TextDisabled("Start: not set");
    if (view.goal)
        ImGui::Text("Goal: (%.1f, %.1f)", view.goal->x, view.goal->y);
    else ImGui::TextDisabled("Goal: not set");
    ImGui::Checkbox("Auto Update Path", &auto_update_path);
    requests.generate_path = ImGui::Button("Generate Path", ImVec2(-1.0f, 24.0f)) &&
        has_baked_mesh && view.start.has_value() && view.goal.has_value();
    requests.clear_path = ImGui::Button("Clear Path", ImVec2(-1.0f, 24.0f));
    if (!has_baked_mesh || !view.start || !view.goal)
        ImGui::TextDisabled("Requires a baked navmesh, start, and goal.");
    if (view.result) {
        const auto& path = *view.result;
        switch (path.status) {
            case pathfinding::PathStatus::Success:
                ImGui::TextColored(path_success_color, "Path generated: %zu waypoints", path.waypoints.size());
                break;
            case pathfinding::PathStatus::StartOutsideMesh:
                ImGui::TextColored(path_failure_color, "Path failed: start is outside traversable mesh.");
                break;
            case pathfinding::PathStatus::GoalOutsideMesh:
                ImGui::TextColored(path_failure_color, "Path failed: goal is outside traversable mesh.");
                break;
            case pathfinding::PathStatus::NoPath:
                ImGui::TextColored(path_failure_color, "Path failed: no traversable route.");
                break;
        }
    }
}

/// Renders all control sections and returns user actions without applying them to app state.
ControlRequests ControlPanel::Draw(const ControlPanelContext& context) {
    ControlRequests requests{};
    const Camera2D& camera = context.camera;
    const Vector2 mouse_screen = context.mouse_screen;
    const Vector2d mouse_world = context.mouse_world;
    const GeometryModel& model = context.model;
    const EditorState& editor = context.editor;
    const BakePanelView& bake = context.bake;

    ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360.0f, 680.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Geometry & Navmesh Controls")) {
        ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Map File");
        ImGui::InputText("Path (.txt)", map_path, sizeof(map_path));
        requests.save_map = ImGui::Button("Save Map", ImVec2(-1.0f, 24.0f));
        requests.load_map = ImGui::Button("Load Map", ImVec2(-1.0f, 24.0f));
        if (requests.save_map || requests.load_map) requests.map_path = map_path;
        if (!map_status.empty()) ImGui::TextWrapped("%s", map_status.c_str());
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Geometry & Editing");
        ImGui::Separator();
        ImGui::Text("Total Points: %zu", model.VertexCount());
        ImGui::Text("Edges: %zu   Closed loops: %zu", model.Edges().size(), model.Loops().size());
        const Vertex* hovered = editor.hovered_vertex ? model.FindVertex(*editor.hovered_vertex) : nullptr;
        if (hovered) {
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f), "Hovered: P%llu (%.1f, %.1f)",
                static_cast<unsigned long long>(hovered->id), hovered->position.x, hovered->position.y);
        } else {
            ImGui::TextDisabled("Hover over a point to inspect");
        }

        ImGui::Separator();
        if (ImGui::RadioButton("Edit", editor.active_tool == EditorTool::Edit))
            requests.selected_tool = EditorTool::Edit;
        if (ImGui::RadioButton("Draw Border", editor.active_tool == EditorTool::Border))
            requests.selected_tool = EditorTool::Border;
        if (ImGui::RadioButton("Draw Obstacle", editor.active_tool == EditorTool::Obstacle))
            requests.selected_tool = EditorTool::Obstacle;
        if (ImGui::RadioButton("Set Start", editor.active_tool == EditorTool::Start))
            requests.selected_tool = EditorTool::Start;
        if (ImGui::RadioButton("Set Goal", editor.active_tool == EditorTool::Goal))
            requests.selected_tool = EditorTool::Goal;
        if (editor.active_tool == EditorTool::Edit) {
            ImGui::TextDisabled("Click empty space to add; right-click a point to delete.");
            ImGui::Indent();
            if (ImGui::RadioButton("Move Vertex", editor.edit_behavior == EditBehavior::MoveVertex))
                requests.selected_edit_behavior = EditBehavior::MoveVertex;
            if (ImGui::RadioButton("Move Chain", editor.edit_behavior == EditBehavior::MoveChain))
                requests.selected_edit_behavior = EditBehavior::MoveChain;
            ImGui::Unindent();
            ImGui::TextDisabled(editor.edit_behavior == EditBehavior::MoveChain
                ? "Drag a point to move its connected edges." : "Drag a point to move it.");
        } else if (editor.active_tool == EditorTool::Border || editor.active_tool == EditorTool::Obstacle) {
            const char* description = editor.active_tool == EditorTool::Border
                ? "Click points in order to draw the map boundary."
                : "Click points in order to draw an obstacle.";
            ImGui::TextDisabled("%s", description);
            ImGui::TextDisabled("Click the first point to close; Esc cancels.");
            if (!editor.edge_chain.empty())
                ImGui::Text("Drawing: %zu points", editor.edge_chain.size());
        } else {
            ImGui::TextDisabled(editor.active_tool == EditorTool::Start
                ? "Click the navmesh to place or replace the start point."
                : "Click the navmesh to place or replace the goal point.");
        }
        if (!editor.status_message.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.32f, 1.0f), "%s", editor.status_message.c_str());
        }
        requests.clear_geometry = ImGui::Button("Clear Geometry", ImVec2(-1.0f, 24.0f));
        ImGui::Separator();
        DrawPathfindingControls(requests, context.pathfinding, bake.has_baked_mesh);
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Navmesh Setup & Bake");
        float slider_radius = static_cast<float>(std::clamp(agent_radius, 0.0, 100.0));
        if (ImGui::SliderFloat("Agent radius (0-100)", &slider_radius, 0.0f, 100.0f, "%.2f")) {
            agent_radius = slider_radius;
            requests.agent_radius = agent_radius;
        }
        double edited_radius = agent_radius;
        if (ImGui::InputDouble("Agent radius value", &edited_radius, 0.1, 1.0, "%.2f")) {
            agent_radius = std::isfinite(edited_radius) ? std::max(0.0, edited_radius) : 0.0;
            requests.agent_radius = agent_radius;
        }
        ImGui::Checkbox("Auto-bake on triangulation input changes", &auto_bake_navmesh);
        requests.bake_navmesh = ImGui::Button("Bake Navmesh", ImVec2(-1.0f, 24.0f));
        requests.delete_navmesh = ImGui::Button("Delete Navmesh", ImVec2(-1.0f, 24.0f));
        ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Bake Status");
        ImGui::Text("Stored mesh: %s", bake.has_baked_mesh ? "yes" : "no");
        if (bake.running) ImGui::Text("Bake: running / queued");
        if (bake.out_of_date) ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.32f, 1.0f), "Mesh is out of date");
        if (!bake.attempted) ImGui::TextDisabled("Last bake: not attempted");
        else switch (bake.last_status) {
            case GenerationStatus::Success: ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "Last bake: success"); break;
            case GenerationStatus::NoActiveBorder: ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.32f, 1.0f), "Last bake: no active border"); break;
            case GenerationStatus::Failed: ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "Last bake: failed"); break;
        }

        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "View & Display");
        ImGui::Text("Zoom: %.2fx", camera.zoom);
        ImGui::Text("Camera Target: (%.1f, %.1f)", camera.target.x, camera.target.y);
        ImGui::Text("Cursor (World): (%.1f, %.1f)", mouse_world.x, mouse_world.y);
        ImGui::Text("Cursor (Screen): (%.0f, %.0f)", mouse_screen.x, mouse_screen.y);
        requests.reset_view = ImGui::Button("Reset View (Origin)", ImVec2(-1.0f, 24.0f));
        ImGui::Checkbox("Show Coordinates", &options.show_coordinates);
        ImGui::SliderFloat("Navmesh line transparency", &options.navmesh_edge_transparency, 0.0f, 100.0f, "%.0f%%");
    }
    ImGui::End();

    return requests;
}

void ControlPanel::SetMapStatus(std::string message) {
    map_status = std::move(message);
}

} // namespace cdt
