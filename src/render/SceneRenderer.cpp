#include "render/SceneRenderer.hpp"

#include "raylib.h"
#include "raymath.h"

#include <cstdio>
#include <string>

namespace cdt {
namespace {

constexpr float point_outer_radius = 8.5f;
constexpr float point_inner_radius = 5.5f;
constexpr Color point_outer_color{15, 18, 24, 255};
constexpr Color point_inner_color{80, 200, 240, 255};
constexpr Color point_hover_color{255, 235, 80, 255};
constexpr Color point_drag_color{255, 120, 90, 255};

Vector2 ToRaylib(Vector2d position) {
    return Vector2{static_cast<float>(position.x), static_cast<float>(position.y)};
}

} // namespace

void SceneRenderer::DrawWorld(const GeometryModel& model, const EditorState& editor) const {
    for (const Vertex& vertex : model.Vertices()) {
        Color color = point_inner_color;
        if (editor.dragged_vertex == vertex.id) color = point_drag_color;
        else if (editor.hovered_vertex == vertex.id) color = point_hover_color;

        const Vector2 position = ToRaylib(vertex.position);
        DrawCircleV(position, point_outer_radius, point_outer_color);
        DrawCircleV(position, point_inner_radius, color);
    }
}

void SceneRenderer::DrawOverlays(const GeometryModel& model, const RenderOptions& options,
    const Camera2D& camera) const {
    if (options.show_vertex_ids || options.show_coordinates) {
        for (const Vertex& vertex : model.Vertices()) {
            const Vector2 screen = GetWorldToScreen2D(ToRaylib(vertex.position), camera);
            if (screen.x < -100.0f || screen.x > static_cast<float>(GetScreenWidth()) + 100.0f ||
                screen.y < -50.0f || screen.y > static_cast<float>(GetScreenHeight()) + 50.0f) continue;

            if (options.show_vertex_ids) {
                const std::string label = "P" + std::to_string(vertex.id);
                DrawText(label.c_str(), static_cast<int>(screen.x) + 10, static_cast<int>(screen.y) - 12, 16, RAYWHITE);
            }
            if (options.show_coordinates) {
                char buffer[64];
                std::snprintf(buffer, sizeof(buffer), "(%.1f, %.1f)", vertex.position.x, vertex.position.y);
                DrawText(buffer, static_cast<int>(screen.x) + 10, static_cast<int>(screen.y) + 6,
                    12, Color{170, 185, 200, 210});
            }
        }
    }

    DrawRectangle(0, GetScreenHeight() - 30, GetScreenWidth(), 30, Color{15, 18, 24, 230});
    DrawText("Space + Left Drag: Pan Canvas | Scroll: Zoom to Mouse | Left-click: Add Point | Left Drag: Move Point | Right-click: Delete Point",
        16, GetScreenHeight() - 22, 14, Color{175, 190, 205, 230});
}

} // namespace cdt
