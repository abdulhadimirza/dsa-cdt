#pragma once

#include "model/GeometryModel.hpp"
#include "validation/GeometryValidator.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace cdt {

// Describes whether a mesh triangle can be crossed or belongs to an obstacle.
enum class TriangleRegion { Traversable, Obstacle };

// Describes whether mesh generation succeeded and whether a border was available.
enum class GenerationStatus { Success, NoActiveBorder, Failed };

// A mesh vertex and the geometry vertex it came from.
struct MeshVertex {
    std::optional<VertexId> source_id;
    Vector2d position{};
};

// A triangle stored as three indices into ConstrainedMesh::vertices.
struct MeshTriangle {
    std::array<std::size_t, 3> vertices{};
    // Neighbor across the edge opposite vertices[i]; empty entries are mesh boundaries.
    std::array<std::optional<std::size_t>, 3> neighbors{};
    TriangleRegion region{TriangleRegion::Traversable};
};

enum class PathStatus { Success, InvalidMesh, StartOutsideMesh, GoalOutsideMesh, NoPath };
struct PathResult {
    PathStatus status{PathStatus::NoPath};
    std::vector<std::size_t> triangleCorridor;
    std::vector<Vector2d> waypoints;
};

// The generated vertices, triangles, and required boundary edges.
struct ConstrainedMesh {
    std::vector<MeshVertex> vertices;
    std::vector<MeshTriangle> triangles;
    std::vector<std::array<std::size_t, 2>> constraints;
};

[[nodiscard]] PathResult FindPath(const ConstrainedMesh& mesh, Vector2d start, Vector2d goal);

// The generated mesh on success, or a message explaining why generation stopped.
struct GenerationResult {
    GenerationStatus status{GenerationStatus::Failed};
    ConstrainedMesh mesh;
    std::string message;
};

// Builds a mesh from the active border and eligible obstacle loops.
class ConstrainedDelaunay {
public:
    // Return the generated mesh, or a status and message if generation cannot finish.
    [[nodiscard]] GenerationResult Generate(const GeometryModel& model,
        const GeometryValidation& validation, double agent_radius = 0.0) const;
};

} // namespace cdt
