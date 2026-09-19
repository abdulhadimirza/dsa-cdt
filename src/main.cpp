#include "raylib.h"
#include "raymath.h"
#include "imgui.h"
#include "rlImGui.h"

#include <vector>
#include <span>
#include <string>
#include <cmath>
#include <cstddef>
#include <algorithm>

namespace cdt {

// Modern C++23 / C++26 style aggregate structures with designated initializers
struct Vertex {
    Vector2 position{0.0f, 0.0f};
    int id{0};
    bool is_constrained{false};
};

struct Edge {
    std::size_t u{0};
    std::size_t v{0};
    bool is_constrained{false};
};

struct Triangle {
    std::size_t a{0};
    std::size_t b{0};
    std::size_t c{0};
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
    std::vector<Triangle> triangles;
};

// Creates a sample hardcoded Constrained Delaunay Triangulation (CDT) mesh
[[nodiscard]] Mesh create_sample_mesh() {
    Mesh mesh;

    // Hardcoded vertices (polygon boundary + internal point)
    mesh.vertices = {
        {.position = {380.0f, 260.0f}, .id = 0, .is_constrained = true},
        {.position = {640.0f, 160.0f}, .id = 1, .is_constrained = true},
        {.position = {900.0f, 240.0f}, .id = 2, .is_constrained = true},
        {.position = {960.0f, 490.0f}, .id = 3, .is_constrained = true},
        {.position = {760.0f, 650.0f}, .id = 4, .is_constrained = true},
        {.position = {480.0f, 630.0f}, .id = 5, .is_constrained = true},
        {.position = {320.0f, 450.0f}, .id = 6, .is_constrained = true},
        {.position = {630.0f, 410.0f}, .id = 7, .is_constrained = false}, // Internal vertex
    };

    // Edges: Constrained boundary segments + Delaunay triangulation interior edges
    mesh.edges = {
        // Outer constrained boundary edges
        {.u = 0, .v = 1, .is_constrained = true},
        {.u = 1, .v = 2, .is_constrained = true},
        {.u = 2, .v = 3, .is_constrained = true},
        {.u = 3, .v = 4, .is_constrained = true},
        {.u = 4, .v = 5, .is_constrained = true},
        {.u = 5, .v = 6, .is_constrained = true},
        {.u = 6, .v = 0, .is_constrained = true},

        // Constrained internal feature edge (obstacle / barrier segment)
        {.u = 0, .v = 7, .is_constrained = true},

        // Regular Delaunay edges
        {.u = 1, .v = 7, .is_constrained = false},
        {.u = 2, .v = 7, .is_constrained = false},
        {.u = 3, .v = 7, .is_constrained = false},
        {.u = 4, .v = 7, .is_constrained = false},
        {.u = 5, .v = 7, .is_constrained = false},
        {.u = 6, .v = 7, .is_constrained = false},
    };

    // Triangles forming the Delaunay triangulation
    mesh.triangles = {
        {.a = 0, .b = 1, .c = 7},
        {.a = 1, .b = 2, .c = 7},
        {.a = 2, .b = 3, .c = 7},
        {.a = 3, .b = 4, .c = 7},
        {.a = 4, .b = 5, .c = 7},
        {.a = 5, .b = 6, .c = 7},
        {.a = 6, .b = 0, .c = 7},
    };

    return mesh;
}

// Helper to draw filled triangle independent of winding order
inline void draw_triangle_safe(Vector2 v1, Vector2 v2, Vector2 v3, Color color) {
    const float cross = (v2.x - v1.x) * (v3.y - v1.y) - (v2.y - v1.y) * (v3.x - v1.x);
    if (cross < 0.0f) {
        DrawTriangle(v1, v3, v2, color);
    } else {
        DrawTriangle(v1, v2, v3, color);
    }
}

} // namespace cdt

int main() {
    constexpr int screen_width = 1280;
    constexpr int screen_height = 800;

    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(screen_width, screen_height, "Dynamic Constrained Delaunay Triangulation (CDT)");
    SetTargetFPS(60);

    // Initialize rlImGui
    rlImGuiSetup(true);

    // Load initial hardcoded CDT mesh
    cdt::Mesh mesh = cdt::create_sample_mesh();

    // Visual options
    bool show_triangle_fills = true;
    bool show_delaunay_edges = true;
    bool show_constrained_edges = true;
    bool show_vertices = true;
    bool show_vertex_ids = true;
    bool allow_dragging = true;

    int dragged_vertex_index = -1;

    // Palette
    constexpr Color background_color   = { 22, 25, 32, 255 };
    constexpr Color triangle_fill_even = { 45, 95, 150, 40 };
    constexpr Color triangle_fill_odd  = { 35, 125, 165, 55 };
    constexpr Color delaunay_edge_color= { 90, 160, 220, 190 };
    constexpr Color constraint_edge_color = { 255, 82, 82, 255 };
    constexpr Color vertex_outer_color = { 15, 18, 24, 255 };
    constexpr Color vertex_regular_color = { 80, 200, 240, 255 };
    constexpr Color vertex_constrained_color = { 255, 120, 90, 255 };
    constexpr Color vertex_hover_color = { 255, 235, 80, 255 };

    while (!WindowShouldClose()) {
        const Vector2 mouse_pos = GetMousePosition();
        const bool mouse_captured_by_ui = ImGui::GetIO().WantCaptureMouse;

        // --- Interaction: Drag Vertices ---
        int hovered_vertex_index = -1;
        constexpr float pick_radius = 14.0f;

        if (!mouse_captured_by_ui) {
            for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
                if (CheckCollisionPointCircle(mouse_pos, mesh.vertices[i].position, pick_radius)) {
                    hovered_vertex_index = static_cast<int>(i);
                    break;
                }
            }

            if (allow_dragging) {
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && hovered_vertex_index != -1) {
                    dragged_vertex_index = hovered_vertex_index;
                }
                if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                    dragged_vertex_index = -1;
                }
                if (dragged_vertex_index != -1 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                    mesh.vertices[static_cast<std::size_t>(dragged_vertex_index)].position = mouse_pos;
                }
            }
        } else {
            dragged_vertex_index = -1;
        }

        // --- Drawing Phase ---
        BeginDrawing();
        ClearBackground(background_color);

        // 1. Draw subtle grid background
        constexpr int grid_spacing = 40;
        for (int x = 0; x < GetScreenWidth(); x += grid_spacing) {
            DrawLine(x, 0, x, GetScreenHeight(), Color{ 255, 255, 255, 10 });
        }
        for (int y = 0; y < GetScreenHeight(); y += grid_spacing) {
            DrawLine(0, y, GetScreenWidth(), y, Color{ 255, 255, 255, 10 });
        }

        // 2. Draw Triangles
        if (show_triangle_fills) {
            for (std::size_t i = 0; i < mesh.triangles.size(); ++i) {
                const auto& tri = mesh.triangles[i];
                const Vector2 p0 = mesh.vertices[tri.a].position;
                const Vector2 p1 = mesh.vertices[tri.b].position;
                const Vector2 p2 = mesh.vertices[tri.c].position;
                const Color fill = (i % 2 == 0) ? triangle_fill_even : triangle_fill_odd;
                cdt::draw_triangle_safe(p0, p1, p2, fill);
            }
        }

        // 3. Draw Delaunay and Constrained Edges
        for (const auto& [u, v, is_constrained] : mesh.edges) {
            if (u >= mesh.vertices.size() || v >= mesh.vertices.size()) continue;

            const Vector2 p1 = mesh.vertices[u].position;
            const Vector2 p2 = mesh.vertices[v].position;

            if (is_constrained && show_constrained_edges) {
                // Highlighted thicker line for constraints
                DrawLineEx(p1, p2, 3.5f, constraint_edge_color);
            } else if (!is_constrained && show_delaunay_edges) {
                // Regular Delaunay edge
                DrawLineEx(p1, p2, 1.8f, delaunay_edge_color);
            }
        }

        // 4. Draw Vertices
        if (show_vertices) {
            for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
                const auto& vert = mesh.vertices[i];
                const bool is_dragged = (static_cast<int>(i) == dragged_vertex_index);
                const bool is_hovered = (static_cast<int>(i) == hovered_vertex_index);

                Color inner_color = vert.is_constrained ? vertex_constrained_color : vertex_regular_color;
                if (is_hovered || is_dragged) {
                    inner_color = vertex_hover_color;
                }

                // Outer boundary ring
                DrawCircleV(vert.position, 8.5f, vertex_outer_color);
                // Inner core
                DrawCircleV(vert.position, 5.5f, inner_color);

                // Optional vertex ID label
                if (show_vertex_ids) {
                    const std::string label = "V" + std::to_string(vert.id);
                    DrawText(label.c_str(), static_cast<int>(vert.position.x) + 10, static_cast<int>(vert.position.y) - 12, 16, RAYWHITE);
                }
            }
        }

        // 5. Status text at the bottom
        DrawText("Dynamic CDT Simulation - Step 1: Mesh Display", 20, GetScreenHeight() - 32, 18, Color{ 200, 210, 225, 200 });

        // --- ImGui Render Phase ---
        rlImGuiBegin();

        ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(340.0f, 440.0f), ImGuiCond_FirstUseEver);

        if (ImGui::Begin("CDT Mesh Controls")) {
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Constrained Delaunay Triangulation");
            ImGui::Separator();

            // Mesh Stats
            std::size_t constrained_edges_count = 0;
            for (const auto& edge : mesh.edges) {
                if (edge.is_constrained) ++constrained_edges_count;
            }
            std::size_t delaunay_edges_count = mesh.edges.size() - constrained_edges_count;

            ImGui::Text("Mesh Statistics:");
            ImGui::BulletText("Vertices: %zu", mesh.vertices.size());
            ImGui::BulletText("Triangles: %zu", mesh.triangles.size());
            ImGui::BulletText("Delaunay Edges: %zu", delaunay_edges_count);
            ImGui::BulletText("Constrained Edges: %zu", constrained_edges_count);

            ImGui::Separator();
            ImGui::Text("Render Layers:");
            ImGui::Checkbox("Triangle Faces", &show_triangle_fills);
            ImGui::Checkbox("Delaunay Edges", &show_delaunay_edges);
            ImGui::Checkbox("Constrained Edges", &show_constrained_edges);
            ImGui::Checkbox("Vertices", &show_vertices);
            ImGui::Checkbox("Vertex Labels", &show_vertex_ids);

            ImGui::Separator();
            ImGui::Text("Interaction:");
            ImGui::Checkbox("Allow Dragging Vertices", &allow_dragging);

            if (hovered_vertex_index != -1) {
                const auto& v = mesh.vertices[static_cast<std::size_t>(hovered_vertex_index)];
                ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f), "Hovered: V%d (%.1f, %.1f)", v.id, v.position.x, v.position.y);
            } else {
                ImGui::TextDisabled("Hover over a vertex to inspect");
            }

            ImGui::Separator();
            if (ImGui::Button("Reset Mesh", ImVec2(-1.0f, 30.0f))) {
                mesh = cdt::create_sample_mesh();
            }

            ImGui::Spacing();
            ImGui::Text("Legend:");
            ImGui::ColorButton("ConstrainedColor", ImVec4(1.0f, 0.32f, 0.32f, 1.0f), ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
            ImGui::SameLine();
            ImGui::Text("Constrained Edge");

            ImGui::ColorButton("DelaunayColor", ImVec4(0.35f, 0.63f, 0.86f, 1.0f), ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
            ImGui::SameLine();
            ImGui::Text("Delaunay Edge");
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

