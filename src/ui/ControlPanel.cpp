#include "ui/ControlPanel.hpp"

#include "imgui.h"

namespace cdt {

ControlRequests ControlPanel::Draw(const Camera2D& camera, Vector2 mouse_screen,
    Vector2d mouse_world, const GeometryModel& model, const EditorState& editor) {
    ControlRequests requests{};

    ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320.0f, 500.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Canvas & Points Controls")) {
        ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Camera & World Space");
        ImGui::Separator();
        ImGui::Text("Zoom: %.2fx", camera.zoom);
        ImGui::Text("Camera Target: (%.1f, %.1f)", camera.target.x, camera.target.y);
        ImGui::Text("Cursor (World): (%.1f, %.1f)", mouse_world.x, mouse_world.y);
        ImGui::Text("Cursor (Screen): (%.0f, %.0f)", mouse_screen.x, mouse_screen.y);
        requests.reset_view = ImGui::Button("Reset View (Origin)", ImVec2(-1.0f, 24.0f));

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Geometry Editor");
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
        ImGui::Checkbox("Show Point IDs", &options.show_vertex_ids);
        ImGui::Checkbox("Show Coordinates", &options.show_coordinates);
        ImGui::Separator();
        if (ImGui::RadioButton("Edit", editor.active_tool == EditorTool::Edit))
            requests.selected_tool = EditorTool::Edit;
        if (ImGui::RadioButton("Draw Border", editor.active_tool == EditorTool::Border))
            requests.selected_tool = EditorTool::Border;
        if (ImGui::RadioButton("Draw Obstacle", editor.active_tool == EditorTool::Obstacle))
            requests.selected_tool = EditorTool::Obstacle;
        if (editor.active_tool == EditorTool::Edit) {
            ImGui::TextDisabled("Click empty space to add; right-click a point to delete.");
            ImGui::Indent();
            if (ImGui::RadioButton("Move Vertex", editor.edit_behavior == EditBehavior::MoveVertex))
                requests.selected_edit_behavior = EditBehavior::MoveVertex;
            if (ImGui::RadioButton("Move Loop", editor.edit_behavior == EditBehavior::MoveLoop))
                requests.selected_edit_behavior = EditBehavior::MoveLoop;
            ImGui::Unindent();
            ImGui::TextDisabled(editor.edit_behavior == EditBehavior::MoveLoop
                ? "Drag a loop point to move its connected loops." : "Drag a point to move it.");
        } else {
            const char* description = editor.active_tool == EditorTool::Border
                ? "Click points in order to draw the map boundary."
                : "Click points in order to draw an obstacle.";
            ImGui::TextDisabled("%s", description);
            ImGui::TextDisabled("Click the first point to close; Esc cancels.");
            if (!editor.edge_chain.empty())
                ImGui::Text("Drawing: %zu points", editor.edge_chain.size());
        }
        if (!editor.status_message.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.32f, 1.0f), "%s", editor.status_message.c_str());
        }
        requests.clear_geometry = ImGui::Button("Clear Geometry", ImVec2(-1.0f, 24.0f));
    }
    ImGui::End();

    return requests;
}

} // namespace cdt
