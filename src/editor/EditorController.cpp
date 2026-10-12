#include "editor/EditorController.hpp"

namespace cdt {

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

void EditorController::ClearInteraction() noexcept {
    state.hovered_vertex.reset();
    state.dragged_vertex.reset();
    state.drag_offset = {};
    state.edge_chain.clear();
}

void EditorController::SetTool(EditorTool tool) noexcept {
    if (state.active_tool == tool) return;
    state.active_tool = tool;
    ClearInteraction();
}

void EditorController::Update(const EditorInput& input, GeometryModel& model) {
    if (input.escape_pressed && state.active_tool != EditorTool::AddMove) {
        state.edge_chain.clear();
    }
    if (!input.editing_allowed) {
        state.hovered_vertex.reset();
        state.dragged_vertex.reset();
        state.drag_offset = {};
        return;
    }

    if (state.dragged_vertex && !model.FindVertex(*state.dragged_vertex)) {
        state.dragged_vertex.reset();
    }

    if (state.active_tool != EditorTool::AddMove) {
        state.hovered_vertex = PickVertex(model, input.cursor_world, input.pick_radius_world);
        if (input.escape_pressed) return;
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
        return;
    }

    if (state.dragged_vertex) {
        state.hovered_vertex = state.dragged_vertex;
        if (input.left_down) {
            model.MoveVertex(*state.dragged_vertex, input.cursor_world + state.drag_offset);
        }
        if (input.left_released) state.dragged_vertex.reset();
    } else {
        state.hovered_vertex = PickVertex(model, input.cursor_world, input.pick_radius_world);
        if (input.left_pressed) {
            if (state.hovered_vertex) {
                state.dragged_vertex = state.hovered_vertex;
                const Vertex* vertex = model.FindVertex(*state.hovered_vertex);
                state.drag_offset = vertex ? vertex->position - input.cursor_world : Vector2d{};
            } else {
                (void)model.AddVertex(input.cursor_world);
            }
        }
    }

    if (input.right_pressed) {
        const auto target = state.dragged_vertex ? state.dragged_vertex : state.hovered_vertex;
        if (target) model.RemoveVertex(*target);
        ClearInteraction();
    }
}

} // namespace cdt
