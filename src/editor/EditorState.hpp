#pragma once

#include "core/Vector2d.hpp"
#include "model/GeometryModel.hpp"

#include <optional>
#include <string>
#include <vector>

namespace cdt {

enum class EditorTool { Edit, Border, Obstacle };
enum class EditBehavior { MoveVertex, MoveLoop };

struct EditorState {
    std::optional<VertexId> hovered_vertex;
    std::optional<VertexId> dragged_vertex;
    Vector2d drag_offset{};
    Vector2d last_drag_cursor{};
    EditorTool active_tool{EditorTool::Edit};
    EditBehavior edit_behavior{EditBehavior::MoveVertex};
    std::vector<VertexId> edge_chain;
    std::vector<VertexId> loop_move_vertices;
    std::string status_message;
};

} // namespace cdt
