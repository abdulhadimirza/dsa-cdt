#include "render/SceneRenderer.hpp"

#include "raylib.h"
#include "raymath.h"

#include <cstdio>
#include <string>
#include <unordered_set>

namespace cdt {
namespace {

constexpr float point_outer_radius = 8.5f;
constexpr float point_inner_radius = 5.5f;
constexpr Color point_outer_color{15, 18, 24, 255};
constexpr Color point_inner_color{80, 200, 240, 255};
constexpr Color point_hover_color{255, 235, 80, 255};
constexpr Color point_drag_color{255, 120, 90, 255};
constexpr Color border_edge_color{255, 190, 70, 255};
constexpr Color obstacle_edge_color{220, 100, 180, 255};
constexpr Color invalid_loop_color{245, 80, 85, 255};
constexpr Color edge_preview_color{220, 235, 245, 170};
constexpr Color chain_move_color{90, 235, 185, 230};

Vector2 ToRaylib(Vector2d position) {
    return Vector2{static_cast<float>(position.x), static_cast<float>(position.y)};
}

} // namespace

void SceneRenderer::DrawWorld(const GeometryModel& model, const EditorState& editor) const {
    const std::unordered_set<VertexId> moving_chain_vertices(
        editor.chain_move_vertices.begin(), editor.chain_move_vertices.end());
    const auto is_loop_edge = [&model](const GeometryEdge& edge) {
        for (const GeometryLoop& loop : model.Loops()) {
            if (loop.type != edge.type) continue;
            for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
                const VertexId a = loop.vertices[i];
                const VertexId b = loop.vertices[(i + 1) % loop.vertices.size()];
                if ((a == edge.first && b == edge.second) || (a == edge.second && b == edge.first)) return true;
            }
        }
        return false;
    };
    for (const GeometryEdge& edge : model.Edges()) {
        if (is_loop_edge(edge)) continue;
        const Vertex* first = model.FindVertex(edge.first);
        const Vertex* second = model.FindVertex(edge.second);
        if (first && second) DrawLineEx(ToRaylib(first->position), ToRaylib(second->position), 3.0f,
            edge.type == EdgeType::Border ? border_edge_color : obstacle_edge_color);
    }
    for (const GeometryLoop& loop : model.Loops()) {
        const Color color = !loop.valid ? invalid_loop_color :
            (loop.type == EdgeType::Border ? border_edge_color : obstacle_edge_color);
        for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
            const Vertex* first = model.FindVertex(loop.vertices[i]);
            const Vertex* second = model.FindVertex(loop.vertices[(i + 1) % loop.vertices.size()]);
            if (first && second) DrawLineEx(ToRaylib(first->position), ToRaylib(second->position), 3.5f, color);
        }
    }
    if (editor.dragged_vertex && editor.edit_behavior == EditBehavior::MoveChain) {
        for (const GeometryEdge& edge : model.Edges()) {
            if (!moving_chain_vertices.contains(edge.first) || !moving_chain_vertices.contains(edge.second)) continue;
            const Vertex* first = model.FindVertex(edge.first);
            const Vertex* second = model.FindVertex(edge.second);
            if (first && second)
                DrawLineEx(ToRaylib(first->position), ToRaylib(second->position), 5.0f, chain_move_color);
        }
    }
    if (!editor.edge_chain.empty()) {
        const Vertex* from = model.FindVertex(editor.edge_chain.back());
        if (from && editor.hovered_vertex) {
            const Vertex* to = model.FindVertex(*editor.hovered_vertex);
            if (to && to->id != from->id)
                DrawLineEx(ToRaylib(from->position), ToRaylib(to->position), 2.0f, edge_preview_color);
        }
    }
    for (const Vertex& vertex : model.Vertices()) {
        Color color = point_inner_color;
        if (moving_chain_vertices.contains(vertex.id) && editor.dragged_vertex &&
            editor.edit_behavior == EditBehavior::MoveChain) color = chain_move_color;
        else if (editor.dragged_vertex == vertex.id) color = point_drag_color;
        else if (editor.hovered_vertex == vertex.id) color = point_hover_color;

        const Vector2 position = ToRaylib(vertex.position);
        DrawCircleV(position, point_outer_radius, point_outer_color);
        DrawCircleV(position, point_inner_radius, color);
    }
}

void SceneRenderer::DrawOverlays(const GeometryModel& model, const RenderOptions& options,
    const Camera2D& camera, const EditorState& editor) const {
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
    DrawText("Space + Left Drag: Pan | Scroll: Zoom | Edit: add / move / delete points | Border and Obstacle: click points in order; Esc cancels",
        16, GetScreenHeight() - 22, 14, Color{175, 190, 205, 230});
    if (!editor.status_message.empty()) {
        DrawText(editor.status_message.c_str(), 16, GetScreenHeight() - 54, 16, Color{255, 190, 90, 255});
    }
}

} // namespace cdt
