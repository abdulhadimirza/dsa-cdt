#pragma once

#include "core/Vector2d.hpp"
#include "navmesh/WalkableRegion.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cdt {

/// Classification used by pathfinding to traverse walkable faces only.
enum class TriangleRegion { Traversable, Obstacle };
/// User-facing outcome of the generation pipeline.
enum class GenerationStatus { Success, NoActiveBorder, Failed };

/// A generated position and its original geometry vertex identity when unchanged.
struct MeshVertex {
    /// Present only when this generated point exactly matches an original geometry vertex.
    std::optional<std::uint64_t> source_id;
    /// World-space position used by the generated mesh.
    Vector2d position{};
};

/// Triangle connectivity; neighbor slot i is across the edge opposite vertex i.
struct MeshTriangle {
    /// Indices into NavMesh::Vertices(), ordered counterclockwise.
    std::array<std::size_t, 3> vertices{};
    /// Neighbor across each edge opposite the matching vertex; null marks a boundary.
    std::array<std::optional<std::size_t>, 3> neighbors{};
    TriangleRegion region{TriangleRegion::Traversable};
};

struct ConstrainedMesh;

/// Immutable validated mesh published only after final adjacency succeeds.
class NavMesh {
public:
    /// Validated meshes can be copied or moved while keeping all public access read-only.
    NavMesh(const NavMesh&) = default;
    NavMesh(NavMesh&&) noexcept = default;
    NavMesh& operator=(const NavMesh&) = default;
    NavMesh& operator=(NavMesh&&) noexcept = default;
    /// Access generated vertices without exposing mutable topology.
    [[nodiscard]] const std::vector<MeshVertex>& Vertices() const noexcept { return vertices_; }
    /// Access generated faces whose neighbor links passed final adjacency validation.
    [[nodiscard]] const std::vector<MeshTriangle>& Triangles() const noexcept { return triangles_; }
    /// Access constrained contour edges as vertex-index pairs.
    [[nodiscard]] const std::vector<std::array<std::size_t, 2>>& Constraints() const noexcept { return constraints_; }
private:
    friend class ConstrainedDelaunay;
    explicit NavMesh(ConstrainedMesh&& draft);
    std::vector<MeshVertex> vertices_;
    std::vector<MeshTriangle> triangles_;
    std::vector<std::array<std::size_t, 2>> constraints_;
};

/// Mutable construction buffer, never exposed as generated output.
struct ConstrainedMesh {
    std::vector<MeshVertex> vertices;
    std::vector<MeshTriangle> triangles;
    std::vector<std::array<std::size_t, 2>> constraints;
};

/// Mesh output is present only after generation and adjacency validation succeed.
struct GenerationResult {
    /// Pipeline outcome; mesh is populated only for Success.
    GenerationStatus status{GenerationStatus::Failed};
    /// Immutable output, created only after all final adjacency checks pass.
    std::optional<NavMesh> mesh;
    /// User-facing diagnostic for failed generation outcomes.
    std::string message;
};

}
