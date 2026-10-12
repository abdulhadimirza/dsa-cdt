#pragma once

#include "core/Vector2d.hpp"
#include "model/GeometryModel.hpp"

#include <optional>
#include <vector>

namespace cdt {

enum class EditorTool { AddMove, Border, Obstacle };

struct EditorState {
    std::optional<VertexId> hovered_vertex;
    std::optional<VertexId> dragged_vertex;
    Vector2d drag_offset{};
    EditorTool active_tool{EditorTool::AddMove};
    std::vector<VertexId> edge_chain;
};

} // namespace cdt
