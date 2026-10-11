#pragma once

#include "core/Vector2d.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace cdt {

using VertexId = std::uint64_t;

struct Vertex {
    VertexId id{};
    Vector2d position{};
};

class GeometryModel {
public:
    [[nodiscard]] VertexId AddVertex(Vector2d position);
    [[nodiscard]] bool MoveVertex(VertexId id, Vector2d position) noexcept;
    [[nodiscard]] bool RemoveVertex(VertexId id) noexcept;
    void Clear() noexcept;
    void ResetVertices(const std::vector<Vector2d>& positions);

    [[nodiscard]] const Vertex* FindVertex(VertexId id) const noexcept;
    [[nodiscard]] const std::vector<Vertex>& Vertices() const noexcept { return vertices; }
    [[nodiscard]] std::size_t VertexCount() const noexcept { return vertices.size(); }

private:
    std::vector<Vertex> vertices;
    VertexId next_id{0};
};

} // namespace cdt
