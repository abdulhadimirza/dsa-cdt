#pragma once

#include "core/Vector2d.hpp"
#include "model/GeometryModel.hpp"

#include <optional>

namespace cdt {

struct EditorState {
    std::optional<VertexId> hovered_vertex;
    std::optional<VertexId> dragged_vertex;
    Vector2d drag_offset{};
};

} // namespace cdt
