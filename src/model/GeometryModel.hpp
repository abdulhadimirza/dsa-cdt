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
    /// Return the current geometry version. It advances after each successful geometry update,
    /// including Replace and Clear, so cached results can tell when they no longer match the model.
    [[nodiscard]] std::uint64_t Version() const noexcept { return version; }

private:
    void RebuildLoops();
    /// Mark one completed geometry update so dependent cached data can be refreshed.
    void AdvanceVersion() noexcept { ++version; }
    std::vector<Vertex> vertices;
    std::vector<GeometryEdge> edges;
    std::vector<GeometryLoop> loops;
    VertexId next_id{0};
    std::uint64_t next_loop_completion_order{0};
    /// Version of the current geometry state, advanced whenever a public operation changes it.
    std::uint64_t version{0};
};

} // namespace cdt
