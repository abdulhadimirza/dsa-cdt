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
    state = {};
}

void EditorController::Update(const EditorInput& input, GeometryModel& model) {
    if (!input.editing_allowed) {
        ClearInteraction();
        return;
    }

    if (state.dragged_vertex && !model.FindVertex(*state.dragged_vertex)) {
        state.dragged_vertex.reset();
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
