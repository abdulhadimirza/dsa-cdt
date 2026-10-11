#include "ui/ControlPanel.hpp"

#include "imgui.h"

namespace cdt {

ControlRequests ControlPanel::Draw(const Camera2D& camera, Vector2 mouse_screen,
    Vector2d mouse_world, const GeometryModel& model, const EditorState& editor) {
    ControlRequests requests{};

    ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300.0f, 380.0f), ImGuiCond_FirstUseEver);

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
        ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Delaunay Point Set");
        ImGui::Separator();
        ImGui::Text("Total Points: %zu", model.VertexCount());
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
        requests.reset_default_vertices = ImGui::Button("Reset Default Points", ImVec2(-1.0f, 24.0f));
        requests.generate_random_vertices = ImGui::Button("Generate Random Points", ImVec2(-1.0f, 24.0f));
        requests.clear_vertices = ImGui::Button("Clear Points", ImVec2(-1.0f, 24.0f));
    }
    ImGui::End();

    return requests;
}

} // namespace cdt
