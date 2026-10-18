#include "cdt/ConstrainedDelaunay.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>

namespace {

void Require(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(EXIT_FAILURE); }
}

void AddLoop(cdt::GeometryModel& model, const std::vector<cdt::Vector2d>& points, cdt::EdgeType type) {
    std::vector<cdt::VertexId> ids;
    for (const auto& point : points) ids.push_back(model.AddVertex(point));
    for (std::size_t i = 0; i < ids.size(); ++i)
        Require(model.AddEdge(ids[i], ids[(i + 1) % ids.size()], type), "Could not create test loop.");
}

bool Contains(const std::vector<cdt::Vector2d>& polygon, cdt::Vector2d point) {
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const auto& a = polygon[i]; const auto& b = polygon[j];
        if ((a.y > point.y) != (b.y > point.y) &&
            point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
}

double InCircle(cdt::Vector2d a, cdt::Vector2d b, cdt::Vector2d c, cdt::Vector2d d) {
    const double ax = a.x - d.x, ay = a.y - d.y;
    const double bx = b.x - d.x, by = b.y - d.y;
    const double cx = c.x - d.x, cy = c.y - d.y;
    return (ax * ax + ay * ay) * (bx * cy - by * cx) -
        (bx * bx + by * by) * (ax * cy - ay * cx) +
        (cx * cx + cy * cy) * (ax * by - ay * bx);
}

void TestNoActiveBorder() {
    cdt::GeometryModel model;
    cdt::GeometryValidator validator;
    const auto result = cdt::ConstrainedDelaunay{}.Generate(model, validator.Validate(model));
    Require(result.status == cdt::GenerationStatus::NoActiveBorder, "Expected no-border status.");
    Require(result.mesh.triangles.empty(), "No-border generation returned triangles.");
}

void TestBorderAndObstacle() {
    cdt::GeometryModel model;
    AddLoop(model, {{0, 0}, {12, 0}, {12, 10}, {0, 10}}, cdt::EdgeType::Border);
    AddLoop(model, {{4, 3}, {8, 3}, {8, 7}, {4, 7}}, cdt::EdgeType::Obstacle);
    cdt::GeometryValidator validator;
    const auto result = cdt::ConstrainedDelaunay{}.Generate(model, validator.Validate(model));
    Require(result.status == cdt::GenerationStatus::Success, "Expected valid CDT generation.");
    Require(result.mesh.vertices.size() == 8, "Not every loop vertex was retained.");
    Require(result.mesh.constraints.size() == 8, "A border or obstacle constraint was lost.");
    bool has_traversable = false, has_obstacle = false;
    std::map<std::array<std::size_t, 2>, std::vector<std::size_t>> edge_faces;
    const std::vector<cdt::Vector2d> border{{0, 0}, {12, 0}, {12, 10}, {0, 10}};
    const std::vector<cdt::Vector2d> obstacle{{4, 3}, {8, 3}, {8, 7}, {4, 7}};
    for (const auto& triangle : result.mesh.triangles) {
        if (triangle.region == cdt::TriangleRegion::Traversable) has_traversable = true;
        if (triangle.region == cdt::TriangleRegion::Obstacle) has_obstacle = true;
        cdt::Vector2d center{};
        for (std::size_t v : triangle.vertices) center += result.mesh.vertices[v].position / 3.0;
        Require(Contains(border, center), "A triangle lies outside the border.");
        Require((triangle.region == cdt::TriangleRegion::Obstacle) == Contains(obstacle, center),
            "Triangle region classification does not match obstacle containment.");
        for (int e = 0; e < 3; ++e) {
            auto a = triangle.vertices[e], b = triangle.vertices[(e + 1) % 3];
            if (a > b) std::swap(a, b);
            edge_faces[{a, b}].push_back(triangle.vertices[(e + 2) % 3]);
        }
    }
    Require(has_traversable && has_obstacle, "Expected both triangle region classes.");
    for (const auto& constraint : result.mesh.constraints)
        Require(edge_faces.contains(constraint), "A constraint is absent from the triangle topology.");
    for (const auto& [edge, opposite] : edge_faces) {
        if (opposite.size() != 2 || std::find(result.mesh.constraints.begin(), result.mesh.constraints.end(), edge)
            != result.mesh.constraints.end()) continue;
        const auto a = result.mesh.vertices[edge[0]].position;
        const auto b = result.mesh.vertices[edge[1]].position;
        const auto c = result.mesh.vertices[opposite[0]].position;
        const auto d = result.mesh.vertices[opposite[1]].position;
        const double area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        double determinant = InCircle(a, b, c, d);
        if (area < 0.0) determinant = -determinant;
        Require(determinant <= 1e-8, "An unconstrained interior edge violates the local Delaunay condition.");
    }
}

} // namespace

int main() {
    TestNoActiveBorder();
    TestBorderAndObstacle();
}
