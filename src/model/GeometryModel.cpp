#include "model/GeometryModel.hpp"

#include <algorithm>

namespace cdt {

VertexId GeometryModel::AddVertex(Vector2d position) {
    const VertexId id = next_id++;
    vertices.push_back(Vertex{id, position});
    return id;
}

bool GeometryModel::MoveVertex(VertexId id, Vector2d position) noexcept {
    const auto it = std::find_if(vertices.begin(), vertices.end(), [id](const Vertex& vertex) {
        return vertex.id == id;
    });
    if (it == vertices.end()) return false;
    it->position = position;
    return true;
}

bool GeometryModel::RemoveVertex(VertexId id) noexcept {
    const auto it = std::find_if(vertices.begin(), vertices.end(), [id](const Vertex& vertex) {
        return vertex.id == id;
    });
    if (it == vertices.end()) return false;
    vertices.erase(it);
    return true;
}

void GeometryModel::Clear() noexcept {
    vertices.clear();
}

void GeometryModel::ResetVertices(const std::vector<Vector2d>& positions) {
    vertices.clear();
    next_id = 0;
    vertices.reserve(positions.size());
    for (const Vector2d position : positions) AddVertex(position);
}

const Vertex* GeometryModel::FindVertex(VertexId id) const noexcept {
    const auto it = std::find_if(vertices.begin(), vertices.end(), [id](const Vertex& vertex) {
        return vertex.id == id;
    });
    return it == vertices.end() ? nullptr : &*it;
}

} // namespace cdt
