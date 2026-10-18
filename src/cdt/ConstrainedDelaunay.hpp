#pragma once

#include "model/GeometryModel.hpp"
#include "validation/GeometryValidator.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace cdt {

// Describes whether a mesh triangle can be crossed or belongs to an obstacle.
enum class TriangleRegion { Traversable, Obstacle };

// Describes whether mesh generation succeeded and whether a border was available.
enum class GenerationStatus { Success, NoActiveBorder, Failed };

// A mesh vertex and the geometry vertex it came from.
struct MeshVertex {
    VertexId source_id{};
    Vector2d position{};
};

// A triangle stored as three indices into ConstrainedMesh::vertices.
struct MeshTriangle {
    std::array<std::size_t, 3> vertices{};
    TriangleRegion region{TriangleRegion::Traversable};
};

// The generated vertices, triangles, and required boundary edges.
struct ConstrainedMesh {
    std::vector<MeshVertex> vertices;
    std::vector<MeshTriangle> triangles;
    std::vector<std::array<std::size_t, 2>> constraints;
};

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
        const GeometryValidation& validation) const;
};

} // namespace cdt
