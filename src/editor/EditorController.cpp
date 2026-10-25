#include "editor/EditorController.hpp"

#include <utility>

namespace cdt {

/// Returns the closest vertex at or within the world-space radius, if one exists.
std::optional<VertexId> EditorController::PickVertex(
    const GeometryModel& model, Vector2d cursor, double radius) noexcept {
    const double max_distance_sq = radius * radius;
    double closest_distance_sq = max_distance_sq;
    std::optional<VertexId> closest;
    for (const Vertex& vertex : model.Vertices()) {
        const double distance_sq = vertex.position.distance_sq(cursor);
        if (distance_sq <= closest_distance_sq) {
            closest_distance_sq = distance_sq;
            closest = vertex.id;
        }
    }
    return closest;
}

/// Resets transient editor interaction state without changing geometry or the selected tool.
void EditorController::ClearInteraction() noexcept {
    state.hovered_vertex.reset();
    state.dragged_vertex.reset();
    state.drag_offset = {};
    state.chain_move_vertices.clear();
    state.edge_chain.clear();
    state.status_message.clear();
}

/// Applies the requested movement mode and cancels state that cannot carry across modes.
void EditorController::SetEditBehavior(EditBehavior behavior) noexcept {
    if (state.edit_behavior == behavior) return;
    state.edit_behavior = behavior;
    state.dragged_vertex.reset();
    state.chain_move_vertices.clear();
    state.status_message.clear();
}

/// Selects a tool and cancels any interaction owned by the previous tool.
void EditorController::SetTool(EditorTool tool) noexcept {
    if (state.active_tool == tool) return;
    state.active_tool = tool;
    ClearInteraction();
}

/// Applies input to the editor and returns a generic canvas click for app-level interpretation.
std::optional<CanvasClick> EditorController::Update(const EditorInput& input, GeometryModel& model) {
    if (input.escape_pressed && state.active_tool != EditorTool::Edit) {
        state.edge_chain.clear();
    }
    if (!input.editing_allowed) {
        state.hovered_vertex.reset();
        state.dragged_vertex.reset();
        state.drag_offset = {};
        state.chain_move_vertices.clear();
        return std::nullopt;
    }

    // Start/goal tools emit the current cursor position on every held frame; other tools remain click-based.
    const bool placing_endpoint = state.active_tool == EditorTool::Start || state.active_tool == EditorTool::Goal;
    const std::optional<CanvasClick> canvas_click = (input.left_pressed || (placing_endpoint && input.left_down))
        ? std::optional<CanvasClick>{CanvasClick{state.active_tool, input.cursor_world}}
        : std::nullopt;

    if (state.dragged_vertex && !model.FindVertex(*state.dragged_vertex)) {
        state.dragged_vertex.reset();
    }

    if (state.active_tool == EditorTool::Start || state.active_tool == EditorTool::Goal) {
        state.hovered_vertex.reset();
        state.status_message.clear();
        return canvas_click;
    }

    if (state.active_tool != EditorTool::Edit) {
        state.hovered_vertex = PickVertex(model, input.cursor_world, input.pick_radius_world);
        if (input.escape_pressed) return canvas_click;
        if (input.left_pressed && state.hovered_vertex) {
            const VertexId selected = *state.hovered_vertex;
            if (state.edge_chain.empty()) {
                state.edge_chain.push_back(selected);
            } else if (selected == state.edge_chain.front()) {
                if (state.edge_chain.size() >= 3) {
                    const EdgeType type = state.active_tool == EditorTool::Border
                        ? EdgeType::Border : EdgeType::Obstacle;
                    if (model.AddEdge(state.edge_chain.back(), selected, type)) state.edge_chain.clear();
                }
            } else if (selected != state.edge_chain.back()) {
                const EdgeType type = state.active_tool == EditorTool::Border
                    ? EdgeType::Border : EdgeType::Obstacle;
                if (model.AddEdge(state.edge_chain.back(), selected, type)) state.edge_chain.push_back(selected);
            }
        }
        return canvas_click;
    }

    if (input.escape_pressed) state.status_message.clear();
    if (state.dragged_vertex) {
        state.hovered_vertex = state.dragged_vertex;
        if (input.left_down) {
            if (state.edit_behavior == EditBehavior::MoveChain) {
                const Vector2d offset = input.cursor_world - state.last_drag_cursor;
                (void)model.MoveVerticesBy(state.chain_move_vertices, offset);
                state.last_drag_cursor = input.cursor_world;
            } else {
                (void)model.MoveVertex(*state.dragged_vertex, input.cursor_world + state.drag_offset);
            }
        }
        if (input.left_released) {
            state.dragged_vertex.reset();
            state.chain_move_vertices.clear();
        }
    } else {
        state.hovered_vertex = PickVertex(model, input.cursor_world, input.pick_radius_world);
        if (input.left_pressed) {
            if (state.hovered_vertex) {
                const Vertex* vertex = model.FindVertex(*state.hovered_vertex);
                state.drag_offset = vertex ? vertex->position - input.cursor_world : Vector2d{};
                state.status_message.clear();
                if (state.edit_behavior == EditBehavior::MoveChain) {
                    state.chain_move_vertices = model.ChainForVertex(*state.hovered_vertex);
                    if (state.chain_move_vertices.empty()) {
                        state.status_message = "This point is not connected to an edge.";
                    } else {
                        state.dragged_vertex = state.hovered_vertex;
                        state.last_drag_cursor = input.cursor_world;
                    }
                } else {
                    state.dragged_vertex = state.hovered_vertex;
                }
            } else {
                (void)model.AddVertex(input.cursor_world);
                state.status_message.clear();
            }
        }
    }

    if (input.right_pressed) {
        const auto target = state.dragged_vertex ? state.dragged_vertex : state.hovered_vertex;
        if (target) (void)model.RemoveVertex(*target);
        ClearInteraction();
    }
    return canvas_click;
}

/// Replaces the transient feedback text shown by editor-facing UI.
void EditorController::SetStatusMessage(std::string message) {
    state.status_message = std::move(message);
}

} // namespace cdt
