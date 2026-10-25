#pragma once

#include "core/Vector2d.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cdt {

using VertexId = std::uint64_t;

enum class EdgeType { Border, Obstacle };

struct GeometryEdge {
    VertexId first{};
    VertexId second{};
    EdgeType type{EdgeType::Border};
};

struct GeometryLoop {
    EdgeType type{EdgeType::Border};
    std::vector<VertexId> vertices;
    std::uint64_t completion_order{};
};

struct Vertex {
    VertexId id{};
    Vector2d position{};
};

class GeometryModel {
public:
    [[nodiscard]] VertexId AddVertex(Vector2d position);
    [[nodiscard]] bool MoveVertex(VertexId id, Vector2d position);
    [[nodiscard]] bool MoveVerticesBy(const std::vector<VertexId>& ids, Vector2d offset);
    [[nodiscard]] std::vector<VertexId> ChainForVertex(VertexId id) const;
    [[nodiscard]] bool RemoveVertex(VertexId id);
    [[nodiscard]] bool AddEdge(VertexId first, VertexId second, EdgeType type);
    /// Replaces geometry after validating IDs, positions, and edge references.
    [[nodiscard]] bool Replace(std::vector<Vertex> new_vertices, std::vector<GeometryEdge> new_edges);
    void Clear() noexcept;

    [[nodiscard]] const Vertex* FindVertex(VertexId id) const noexcept;
    [[nodiscard]] const std::vector<Vertex>& Vertices() const noexcept { return vertices; }
    [[nodiscard]] const std::vector<GeometryEdge>& Edges() const noexcept { return edges; }
    [[nodiscard]] const std::vector<GeometryLoop>& Loops() const noexcept { return loops; }
    [[nodiscard]] std::size_t VertexCount() const noexcept { return vertices.size(); }

private:
    void RebuildLoops();
    std::vector<Vertex> vertices;
    std::vector<GeometryEdge> edges;
    std::vector<GeometryLoop> loops;
    VertexId next_id{0};
    std::uint64_t next_loop_completion_order{0};
};

} // namespace cdt
