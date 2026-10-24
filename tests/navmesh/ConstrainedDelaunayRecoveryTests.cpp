#include "core/GeometryPredicates.hpp"
#include "model/GeometryModel.hpp"
#include "navmesh/NavmeshPipeline.hpp"
#include "validation/GeometryValidator.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace cdt;

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void AddLoop(GeometryModel& model, const std::vector<Vector2d>& points, EdgeType type) {
    std::vector<VertexId> ids;
    for (const Vector2d& point : points) ids.push_back(model.AddVertex(point));
    for (std::size_t i = 0; i < ids.size(); ++i)
        Require(model.AddEdge(ids[i], ids[(i + 1) % ids.size()], type), "Could not add fixture loop edge");
}

GeometryModel MakeNearParallelFixture(bool includeCollinearBorderPoint = false) {
    GeometryModel model;
    if (includeCollinearBorderPoint) {
        AddLoop(model, {{0, 0}, {100, 0}, {100, 50}, {100, 100}, {0, 100}}, EdgeType::Border);
    } else {
        AddLoop(model, {{0, 0}, {100, 0}, {100, 100}, {0, 100}}, EdgeType::Border);
    }
    // The obstacle's right edge is parallel to and 3.99 units from the border.
    // Radius 1.99 leaves about 0.01 units of walkable clearance after offsetting.
    AddLoop(model, {{95.5, 20}, {96.01, 20}, {96.01, 80}, {95.5, 80}}, EdgeType::Obstacle);
    return model;
}

void VerifyGenerated(const GeometryModel& model, double radius) {
    const GeometryValidationReport report = GeometryValidator{}.Validate(model);
    Require(report.validated_navmesh_input.has_value(), "Fixture did not pass geometry validation");
    const GenerationResult generated = NavmeshPipeline{}.Generate(report.validated_navmesh_input, radius);
    Require(generated.status == GenerationStatus::Success && generated.mesh.has_value(), generated.message.c_str());
    const NavMesh& mesh = *generated.mesh;
    Require(!mesh.Triangles().empty(), "Generation returned no triangles");

    std::set<std::array<std::size_t, 2>> triangleEdges;
    for (std::size_t i = 0; i < mesh.Triangles().size(); ++i) {
        const MeshTriangle& triangle = mesh.Triangles()[i];
        const auto& a = mesh.Vertices()[triangle.vertices[0]].position;
        const auto& b = mesh.Vertices()[triangle.vertices[1]].position;
        const auto& c = mesh.Vertices()[triangle.vertices[2]].position;
        Require(geometry::Orient2D(a, b, c) > 0.0, "Generated a zero-area or clockwise triangle");
        for (std::size_t slot = 0; slot < 3; ++slot) {
            const std::size_t first = triangle.vertices[(slot + 1) % 3];
            const std::size_t second = triangle.vertices[(slot + 2) % 3];
            triangleEdges.insert({std::min(first, second), std::max(first, second)});
            if (!triangle.neighbors[slot]) continue;
            const std::size_t neighborIndex = *triangle.neighbors[slot];
            Require(neighborIndex < mesh.Triangles().size(), "Neighbor index is out of range");
            const MeshTriangle& neighbor = mesh.Triangles()[neighborIndex];
            bool reciprocal = false;
            for (std::size_t neighborSlot = 0; neighborSlot < 3; ++neighborSlot)
                if (neighbor.neighbors[neighborSlot] == i &&
                    std::min(neighbor.vertices[(neighborSlot + 1) % 3], neighbor.vertices[(neighborSlot + 2) % 3]) == std::min(first, second) &&
                    std::max(neighbor.vertices[(neighborSlot + 1) % 3], neighbor.vertices[(neighborSlot + 2) % 3]) == std::max(first, second))
                    reciprocal = true;
            Require(reciprocal, "Triangle adjacency is not reciprocal across a shared edge");
        }
    }
    for (const auto& constraint : mesh.Constraints())
        Require(triangleEdges.contains({std::min(constraint[0], constraint[1]), std::max(constraint[0], constraint[1])}),
            "A contour constraint is absent from the generated mesh");
}

void RunFixture(const char* name, const GeometryModel& model, double radius) {
    try {
        VerifyGenerated(model, radius);
    } catch (const std::exception& error) {
        throw std::runtime_error(std::string(name) + ": " + error.what());
    }
}

} // namespace

int main() {
    try {
        RunFixture("near-parallel narrow gap (radius 1.99)", MakeNearParallelFixture(), 1.99);
        RunFixture("wider-gap control (radius 1.5)", MakeNearParallelFixture(), 1.5);
        // This border edge contains an existing collinear vertex, exercising the
        // pipeline's contour normalization and constraint recovery around it.
        RunFixture("collinear border vertex (radius 1.5)", MakeNearParallelFixture(true), 1.5);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
