#pragma once

#include "model/GeometryModel.hpp"
#include "validation/GeometryValidator.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace cdt {

enum class TriangleRegion { Traversable, Obstacle };
enum class GenerationStatus { Success, NoActiveBorder, Failed };

struct MeshVertex {
    VertexId source_id{};
    Vector2d position{};
};

struct MeshTriangle {
    std::array<std::size_t, 3> vertices{};
    TriangleRegion region{TriangleRegion::Traversable};
};

struct ConstrainedMesh {
    std::vector<MeshVertex> vertices;
    std::vector<MeshTriangle> triangles;
    std::vector<std::array<std::size_t, 2>> constraints;
};

struct GenerationResult {
    GenerationStatus status{GenerationStatus::Failed};
    ConstrainedMesh mesh;
    std::string message;
};

// Builds a complete constrained triangulation from validator-selected loops.
class ConstrainedDelaunay {
public:
    [[nodiscard]] GenerationResult Generate(const GeometryModel& model,
        const GeometryValidation& validation) const;
};

} // namespace cdt
