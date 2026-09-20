#include "raylib.h"
#include "raymath.h"
#include "imgui.h"
#include "rlImGui.h"

#include "core/Vector2d.hpp"
#include "app/CameraController.hpp"

#include <vector>
#include <string>
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace cdt {

struct Point {
    Vector2d position{0.0, 0.0};
    int id{0};
};

} // namespace cdt

int main() {
    constexpr int screen_width = 1280;
    constexpr int screen_height = 800;

    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(screen_width, screen_height, "DCDT Simulation - Infinite Canvas");
    SetTargetFPS(60);

    // Initialize rlImGui
    rlImGuiSetup(true);

    // Camera Controller for world vs screen space decoupling
    cdt::CameraController camera_controller(Vector2{0.0f, 0.0f});

    // Initial set of points in World Coordinates (centered around origin)
    std::vector<cdt::Point> points = {
        {{ -380.0, -180.0 }, 0},
        {{ -180.0, -240.0 }, 1},
        {{   80.0, -220.0 }, 2},
        {{  320.0, -150.0 }, 3},
        {{  420.0,   80.0 }, 4},
        {{  220.0,  240.0 }, 5},
        {{ -100.0,  220.0 }, 6},
        {{ -330.0,  120.0 }, 7},
        {{ -150.0,  -30.0 }, 8},
        {{   90.0,   10.0 }, 9},
    };
    int next_point_id = static_cast<int>(points.size());

    // Interaction & display flags
    int hovered_point_index = -1;
    int dragged_point_index = -1;
    cdt::Vector2d drag_offset_world{0.0, 0.0};
    bool show_point_ids = false;
    bool show_coordinates = false;

    // Point geometry & interaction constants (World Space)
    constexpr float point_outer_radius = 8.5f;
    constexpr float point_inner_radius = 5.5f;
    constexpr double base_pick_radius_world = point_outer_radius; // Exact pixel-perfect match to visual outer circle
    constexpr double min_pick_radius_screen = 8.0;                // Accessibility floor when zoomed far out

    // Color palette
    constexpr Color background_color  = { 22, 25, 32, 255 };
    constexpr Color point_outer_color = { 15, 18, 24, 255 };
    constexpr Color point_inner_color = { 80, 200, 240, 255 };
    constexpr Color point_hover_color = { 255, 235, 80, 255 };
    constexpr Color point_drag_color  = { 255, 120, 90, 255 };

    while (!WindowShouldClose()) {
        const Vector2 mouse_screen_pos = GetMousePosition();
        const bool mouse_captured = ImGui::GetIO().WantCaptureMouse;

        // 1. Camera Update (Only if ImGui is not capturing mouse)
        if (!mouse_captured) {
            camera_controller.Update();
        }

        const cdt::Vector2d mouse_world_pos = camera_controller.ScreenToWorld(mouse_screen_pos);
        const bool is_panning = camera_controller.IsPanning() || IsKeyDown(KEY_SPACE);

        // 2. Entity Interaction Handling (World-Space)
        if (mouse_captured || is_panning) {
            hovered_point_index = -1;
            dragged_point_index = -1;
        } else {
            // Safety bounds check for dragged point
            if (dragged_point_index >= static_cast<int>(points.size())) {
                dragged_point_index = -1;
            }

            // State A: A point is actively being dragged
            if (dragged_point_index != -1) {
                // Keep hovered state locked to the dragged point
                hovered_point_index = dragged_point_index;

                if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                    points[static_cast<std::size_t>(dragged_point_index)].position = mouse_world_pos + drag_offset_world;
                }
                if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                    dragged_point_index = -1;
                }
            } else {
                // State B: Idle / Hovering (search nearest point in world space)
                const double effective_world_radius = std::max(
                    base_pick_radius_world,
                    min_pick_radius_screen / camera_controller.GetZoom()
                );
                const double max_allowed_dist_sq = effective_world_radius * effective_world_radius;
                double min_dist_sq = max_allowed_dist_sq;
                int closest_index = -1;

                for (int i = 0; i < static_cast<int>(points.size()); ++i) {
                    const cdt::Vector2d diff = mouse_world_pos - points[static_cast<std::size_t>(i)].position;
                    const double dist_sq = diff.dot(diff); // Self dot-product avoids sqrt
                    if (dist_sq <= min_dist_sq) {
                        min_dist_sq = dist_sq;
                        closest_index = i;
                    }
                }
                hovered_point_index = closest_index;

                // Left-click: drag existing point or create a new one in empty space
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                    if (hovered_point_index != -1) {
                        dragged_point_index = hovered_point_index;
                        drag_offset_world = points[static_cast<std::size_t>(hovered_point_index)].position - mouse_world_pos;
                    } else {
                        points.push_back({mouse_world_pos, next_point_id++});
                    }
                }
            }

            // Right-click deletion: reliably delete dragged point if dragging, or hovered point if idle
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
                const int target_to_delete = (dragged_point_index != -1) ? dragged_point_index : hovered_point_index;
                if (target_to_delete != -1 && target_to_delete < static_cast<int>(points.size())) {
                    points.erase(points.begin() + target_to_delete);
                    dragged_point_index = -1;
                    hovered_point_index = -1;
                }
            }
        }

        // --- Drawing Phase ---
        BeginDrawing();
        ClearBackground(background_color);

        // ==========================================
        // World Space Rendering (Inside Camera Mode)
        // ==========================================
        BeginMode2D(camera_controller.GetCamera());
        {
            // 1. Infinite World Grid & Coordinate Axes
            camera_controller.DrawWorldGrid();

            // 2. Draw World-Space Points
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

                const Vector2 pos_v2 = pt.position.to_raylib();
                DrawCircleV(pos_v2, point_outer_radius, point_outer_color);
                DrawCircleV(pos_v2, point_inner_radius, current_color);
            }
        }
        EndMode2D();

        // ==========================================
        // Screen Space Rendering (HUD & Overlays)
        // ==========================================

        // 1. Draw point labels & coordinates in screen space for crisp, unscaled typography
        if (show_point_ids || show_coordinates) {
            for (const auto& pt : points) {
                const Vector2 screen_pt = camera_controller.WorldToScreen(pt.position);

                // Cull off-screen labels
                if (screen_pt.x < -100.0f || screen_pt.x > static_cast<float>(GetScreenWidth()) + 100.0f ||
                    screen_pt.y < -50.0f  || screen_pt.y > static_cast<float>(GetScreenHeight()) + 50.0f) {
                    continue;
                }

                if (show_point_ids) {
                    const std::string label = "P" + std::to_string(pt.id);
                    DrawText(label.c_str(), static_cast<int>(screen_pt.x) + 10, static_cast<int>(screen_pt.y) - 12, 16, RAYWHITE);
                }

                if (show_coordinates) {
                    char buffer[64];
                    std::snprintf(buffer, sizeof(buffer), "(%.1f, %.1f)", pt.position.x, pt.position.y);
                    DrawText(buffer, static_cast<int>(screen_pt.x) + 10, static_cast<int>(screen_pt.y) + 6, 12, Color{ 170, 185, 200, 210 });
                }
            }
        }

        // 2. Status & Navigation bar at the bottom
        DrawRectangle(0, GetScreenHeight() - 30, GetScreenWidth(), 30, Color{ 15, 18, 24, 230 });
        DrawText("Space + Left Drag: Pan Canvas | Scroll: Zoom to Mouse | Left-click: Add Point | Left Drag: Move Point | Right-click: Delete Point",
                 16, GetScreenHeight() - 22, 14, Color{ 175, 190, 205, 230 });

        // --- ImGui Interface ---
        rlImGuiBegin();

        ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300.0f, 380.0f), ImGuiCond_FirstUseEver);

        if (ImGui::Begin("Canvas & Points Controls")) {
            // Camera & Coordinate Section
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Camera & World Space");
            ImGui::Separator();

            ImGui::Text("Zoom: %.2fx", camera_controller.GetZoom());
            const Vector2 cam_target = camera_controller.GetTarget();
            ImGui::Text("Camera Target: (%.1f, %.1f)", cam_target.x, cam_target.y);
            ImGui::Text("Cursor (World): (%.1f, %.1f)", mouse_world_pos.x, mouse_world_pos.y);
            ImGui::Text("Cursor (Screen): (%.0f, %.0f)", mouse_screen_pos.x, mouse_screen_pos.y);

            if (ImGui::Button("Reset View (Origin)", ImVec2(-1.0f, 24.0f))) {
                camera_controller.ResetView(Vector2{0.0f, 0.0f});
            }

            ImGui::Spacing();
            ImGui::Separator();

            // Point Set Section
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Delaunay Point Set");
            ImGui::Separator();

            ImGui::Text("Total Points: %zu", points.size());
            if (hovered_point_index != -1 && hovered_point_index < static_cast<int>(points.size())) {
                const auto& p = points[static_cast<std::size_t>(hovered_point_index)];
                ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f), "Hovered: P%d (%.1f, %.1f)", p.id, p.position.x, p.position.y);
            } else {
                ImGui::TextDisabled("Hover over a point to inspect");
            }

            ImGui::Separator();
            ImGui::Checkbox("Show Point IDs", &show_point_ids);
            ImGui::Checkbox("Show Coordinates", &show_coordinates);

            ImGui::Separator();
            if (ImGui::Button("Reset Default Points", ImVec2(-1.0f, 24.0f))) {
                points = {
                    {{ -380.0, -180.0 }, 0},
                    {{ -180.0, -240.0 }, 1},
                    {{   80.0, -220.0 }, 2},
                    {{  320.0, -150.0 }, 3},
                    {{  420.0,   80.0 }, 4},
                    {{  220.0,  240.0 }, 5},
                    {{ -100.0,  220.0 }, 6},
                    {{ -330.0,  120.0 }, 7},
                    {{ -150.0,  -30.0 }, 8},
                    {{   90.0,   10.0 }, 9},
                };
                next_point_id = 10;
                hovered_point_index = -1;
                dragged_point_index = -1;
            }

            if (ImGui::Button("Generate Random Points", ImVec2(-1.0f, 24.0f))) {
                points.clear();
                hovered_point_index = -1;
                dragged_point_index = -1;
                constexpr int count = 15;
                for (int i = 0; i < count; ++i) {
                    double px = static_cast<double>(GetRandomValue(-450, 450));
                    double py = static_cast<double>(GetRandomValue(-300, 300));
                    points.push_back({{px, py}, next_point_id++});
                }
            }

            if (ImGui::Button("Clear Points", ImVec2(-1.0f, 24.0f))) {
                points.clear();
                hovered_point_index = -1;
                dragged_point_index = -1;
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
