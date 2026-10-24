#include "render/SceneRenderer.hpp"

#include "raylib.h"
#include "raymath.h"

#include <cstdio>
#include <unordered_set>

namespace cdt {
namespace {

constexpr float point_outer_radius = 8.5f;
constexpr float point_inner_radius = 5.5f;
constexpr Color point_outer_color{15, 18, 24, 255};
constexpr Color point_inner_color{80, 200, 240, 255};
constexpr Color point_hover_color{255, 235, 80, 255};
constexpr Color point_drag_color{255, 120, 90, 255};
constexpr Color border_edge_color{75, 205, 160, 255};
constexpr Color obstacle_edge_color{255, 155, 75, 255};
constexpr Color inactive_border_color{125, 135, 150, 255};
constexpr Color inactive_obstacle_color{125, 135, 150, 255};
constexpr Color invalid_loop_color{245, 80, 85, 255};
constexpr Color edge_preview_color{220, 235, 245, 170};
constexpr Color chain_move_color{90, 235, 185, 230};
constexpr Color traversable_mesh_edge{100, 195, 255, 235};
constexpr Color obstacle_mesh_edge{255, 155, 75, 245};
constexpr Color path_line_color{255, 225, 80, 255};
constexpr Color path_waypoint_color{255, 245, 170, 255};
constexpr Color start_marker_color{70, 220, 125, 255};
constexpr Color goal_marker_color{255, 105, 95, 255};

/// Converts a double-precision world position to raylib's float vector representation.
Vector2 ToRaylib(Vector2d position) {
    return Vector2{static_cast<float>(position.x), static_cast<float>(position.y)};
}

/// Draws the successful route and its waypoints from a read-only pathfinding view.
void DrawPath(const pathfinding::PathfindingView& view) {
    const auto& result = view.result;
    if (!result || result->status != pathfinding::PathStatus::Success) return;
    for (std::size_t i = 1; i < result->waypoints.size(); ++i)
        DrawLineEx(ToRaylib(result->waypoints[i - 1]), ToRaylib(result->waypoints[i]), 4.0f, path_line_color);
    for (const Vector2d waypoint : result->waypoints)
        DrawCircleV(ToRaylib(waypoint), 4.0f, path_waypoint_color);
}

/// Draws one labeled endpoint marker using the supplied role-specific color.
void DrawEndpoint(Vector2d position, const char* label, Color marker_color, int label_offset) {
    const Vector2 point = ToRaylib(position);
    DrawCircleV(point, 11.0f, point_outer_color);
    DrawCircleV(point, 7.0f, marker_color);
    DrawText(label, static_cast<int>(point.x) + label_offset, static_cast<int>(point.y) - 8, 16, point_outer_color);
}

/// Draws whichever endpoint markers are present in the current view.
void DrawEndpoints(const pathfinding::PathfindingView& pathfinding) {
    if (pathfinding.start) DrawEndpoint(*pathfinding.start, "S", start_marker_color, -4);
    if (pathfinding.goal) DrawEndpoint(*pathfinding.goal, "G", goal_marker_color, -5);
}

} // namespace

/// Renders world-space geometry, validation feedback, a baked mesh, and the pathfinding overlay.
void SceneRenderer::DrawWorld(const GeometryModel& model, const GeometryValidationReport& validation,
    const EditorState& editor, const RenderOptions& options, const NavMesh* mesh,
    const pathfinding::PathfindingView& pathfinding) const {
    // Diagnostics are indexed in model-loop order by GeometryValidationReport.
    if (mesh) for (const MeshTriangle& triangle : mesh->Triangles()) {
        const Vector2 a = ToRaylib(mesh->Vertices()[triangle.vertices[0]].position);
        const Vector2 b = ToRaylib(mesh->Vertices()[triangle.vertices[1]].position);
        const Vector2 c = ToRaylib(mesh->Vertices()[triangle.vertices[2]].position);
        const bool obstacle = triangle.region == TriangleRegion::Obstacle;
        Color edge = obstacle ? obstacle_mesh_edge : traversable_mesh_edge;
        edge.a = static_cast<unsigned char>(edge.a * (1.0f - options.navmesh_edge_transparency / 100.0f));
        DrawLineEx(a, b, 1.5f, edge);
        DrawLineEx(b, c, 1.5f, edge);
        DrawLineEx(c, a, 1.5f, edge);
    }
    DrawPath(pathfinding);
    DrawEndpoints(pathfinding);
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
    for (std::size_t loop_index = 0; loop_index < model.Loops().size(); ++loop_index) {
        const GeometryLoop& loop = model.Loops()[loop_index];
        const LoopValidation* state = loop_index < validation.loops.size() ? &validation.loops[loop_index] : nullptr;
        // Invalid, inactive, and conflicting loops remain visible with distinct diagnostic colors.
        Color color = invalid_loop_color;
        if (state && state->individually_valid) {
            if (loop.type == EdgeType::Border) color = state && state->active_border
                ? border_edge_color : inactive_border_color;
            else color = state && state->eligible ? obstacle_edge_color : inactive_obstacle_color;
        }
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

/// Renders screen-space coordinate labels, navigation help, and editor status feedback.
void SceneRenderer::DrawOverlays(const GeometryModel& model, const RenderOptions& options,
    const Camera2D& camera, const EditorState& editor) const {
    if (options.show_coordinates) {
        for (const Vertex& vertex : model.Vertices()) {
            const Vector2 screen = GetWorldToScreen2D(ToRaylib(vertex.position), camera);
            if (screen.x < -100.0f || screen.x > static_cast<float>(GetScreenWidth()) + 100.0f ||
                screen.y < -50.0f || screen.y > static_cast<float>(GetScreenHeight()) + 50.0f) continue;

            char buffer[64];
            std::snprintf(buffer, sizeof(buffer), "(%.1f, %.1f)", vertex.position.x, vertex.position.y);
            DrawText(buffer, static_cast<int>(screen.x) + 10, static_cast<int>(screen.y) + 6,
                12, Color{170, 185, 200, 210});
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
