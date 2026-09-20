#include "raylib.h"
#include "raymath.h"
#include "imgui.h"
#include "rlImGui.h"

#include <vector>
#include <string>

namespace cdt {

struct Point {
    Vector2 position{0.0f, 0.0f};
    int id{0};
};

} // namespace cdt

int main() {
    constexpr int screen_width = 1280;
    constexpr int screen_height = 800;

    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(screen_width, screen_height, "Delaunay Triangulation - Points");
    SetTargetFPS(60);

    // Initialize rlImGui
    rlImGuiSetup(true);

    // Initial set of points for triangulation
    std::vector<cdt::Point> points = {
        {{260.0f, 220.0f}, 0},
        {{460.0f, 160.0f}, 1},
        {{720.0f, 180.0f}, 2},
        {{960.0f, 250.0f}, 3},
        {{1060.0f, 480.0f}, 4},
        {{860.0f, 640.0f}, 5},
        {{540.0f, 620.0f}, 6},
        {{310.0f, 520.0f}, 7},
        {{490.0f, 370.0f}, 8},
        {{730.0f, 410.0f}, 9},
    };
    int next_point_id = static_cast<int>(points.size());

    // Interaction & display flags
    int hovered_point_index = -1;
    int dragged_point_index = -1;
    bool show_point_ids = false;
    bool show_coordinates = false;

    // Color palette
    constexpr Color background_color  = { 22, 25, 32, 255 };
    constexpr Color point_outer_color = { 15, 18, 24, 255 };
    constexpr Color point_inner_color = { 80, 200, 240, 255 };
    constexpr Color point_hover_color = { 255, 235, 80, 255 };
    constexpr Color point_drag_color  = { 255, 120, 90, 255 };

    while (!WindowShouldClose()) {
        const Vector2 mouse_pos = GetMousePosition();
        const bool mouse_captured = ImGui::GetIO().WantCaptureMouse;

        // --- Interaction Handling ---
        constexpr float pick_radius = 12.0f;
        hovered_point_index = -1;

        if (!mouse_captured) {
            // Check for hovered point
            for (int i = 0; i < static_cast<int>(points.size()); ++i) {
                if (CheckCollisionPointCircle(mouse_pos, points[static_cast<std::size_t>(i)].position, pick_radius)) {
                    hovered_point_index = i;
                    break;
                }
            }

            // Drag points
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && hovered_point_index != -1) {
                dragged_point_index = hovered_point_index;
            }
            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                dragged_point_index = -1;
            }
            if (dragged_point_index != -1 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                points[static_cast<std::size_t>(dragged_point_index)].position = mouse_pos;
            }

            // Add new point on left click in empty space
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && hovered_point_index == -1 && dragged_point_index == -1) {
                points.push_back({mouse_pos, next_point_id++});
            }

            // Remove point on right click
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && hovered_point_index != -1) {
                points.erase(points.begin() + hovered_point_index);
                if (dragged_point_index == hovered_point_index) {
                    dragged_point_index = -1;
                } else if (dragged_point_index > hovered_point_index) {
                    --dragged_point_index;
                }
            }
        } else {
            dragged_point_index = -1;
        }

        // --- Drawing Phase ---
        BeginDrawing();
        ClearBackground(background_color);

        // 1. Subtle background grid
        constexpr int grid_spacing = 40;
        for (int x = 0; x < GetScreenWidth(); x += grid_spacing) {
            DrawLine(x, 0, x, GetScreenHeight(), Color{ 255, 255, 255, 10 });
        }
        for (int y = 0; y < GetScreenHeight(); y += grid_spacing) {
            DrawLine(0, y, GetScreenWidth(), y, Color{ 255, 255, 255, 10 });
        }

        // 2. Draw Points
        for (std::size_t i = 0; i < points.size(); ++i) {
            const auto& pt = points[i];
            const bool is_dragged = (static_cast<int>(i) == dragged_point_index);
            const bool is_hovered = (static_cast<int>(i) == hovered_point_index);

            Color current_color = point_inner_color;
            if (is_dragged) {
                current_color = point_drag_color;
            } else if (is_hovered) {
                current_color = point_hover_color;
            }

            // Outer ring and inner filled circle
            DrawCircleV(pt.position, 8.5f, point_outer_color);
            DrawCircleV(pt.position, 5.5f, current_color);

            // Point ID label
            if (show_point_ids) {
                const std::string label = "P" + std::to_string(pt.id);
                DrawText(label.c_str(), static_cast<int>(pt.position.x) + 10, static_cast<int>(pt.position.y) - 12, 16, RAYWHITE);
            }

            // Optional coordinates
            if (show_coordinates) {
                const std::string coords = "(" + std::to_string(static_cast<int>(pt.position.x)) + ", " + std::to_string(static_cast<int>(pt.position.y)) + ")";
                DrawText(coords.c_str(), static_cast<int>(pt.position.x) + 10, static_cast<int>(pt.position.y) + 6, 12, Color{ 170, 185, 200, 210 });
            }
        }

        // 3. Status instructions at the bottom
        DrawText("Left-click: Add point | Left-click & Drag: Move point | Right-click: Delete point", 20, GetScreenHeight() - 28, 15, Color{ 160, 175, 190, 200 });

        // --- ImGui Interface ---
        rlImGuiBegin();

        ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(280.0f, 280.0f), ImGuiCond_FirstUseEver);

        if (ImGui::Begin("Points Controls")) {
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Delaunay Point Set");
            ImGui::Separator();

            ImGui::Text("Total Points: %zu", points.size());
            if (hovered_point_index != -1) {
                const auto& p = points[static_cast<std::size_t>(hovered_point_index)];
                ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f), "Hovered: P%d (%.1f, %.1f)", p.id, p.position.x, p.position.y);
            } else {
                ImGui::TextDisabled("Hover over a point to inspect");
            }

            ImGui::Separator();
            ImGui::Checkbox("Show Point IDs", &show_point_ids);
            ImGui::Checkbox("Show Coordinates", &show_coordinates);

            ImGui::Separator();
            if (ImGui::Button("Reset Default Points", ImVec2(-1.0f, 26.0f))) {
                points = {
                    {{260.0f, 220.0f}, 0},
                    {{460.0f, 160.0f}, 1},
                    {{720.0f, 180.0f}, 2},
                    {{960.0f, 250.0f}, 3},
                    {{1060.0f, 480.0f}, 4},
                    {{860.0f, 640.0f}, 5},
                    {{540.0f, 620.0f}, 6},
                    {{310.0f, 520.0f}, 7},
                    {{490.0f, 370.0f}, 8},
                    {{730.0f, 410.0f}, 9},
                };
                next_point_id = 10;
            }

            if (ImGui::Button("Generate Random Points", ImVec2(-1.0f, 26.0f))) {
                points.clear();
                constexpr int count = 15;
                for (int i = 0; i < count; ++i) {
                    float px = static_cast<float>(GetRandomValue(100, GetScreenWidth() - 100));
                    float py = static_cast<float>(GetRandomValue(100, GetScreenHeight() - 100));
                    points.push_back({{px, py}, next_point_id++});
                }
            }

            if (ImGui::Button("Clear Points", ImVec2(-1.0f, 26.0f))) {
                points.clear();
            }
        }
        ImGui::End();

        rlImGuiEnd();

        EndDrawing();
    }

    // Cleanup
    rlImGuiShutdown();
    CloseWindow();

    return 0;
}
