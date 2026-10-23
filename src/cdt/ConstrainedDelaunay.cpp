#include "cdt/ConstrainedDelaunay.hpp"
#include "clipper2/clipper.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <stddef.h>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

#include "robust-predicates/geompred.hpp"

namespace cdt {
namespace detail {

using Edge = std::array<std::size_t, 2>;
using Triangle = std::array<std::size_t, 3>;
using TriangleIndex = std::size_t;

struct StoredTriangle {
    Triangle vertexIndices{};
    std::array<std::optional<TriangleIndex>, 3> neighborTriangleIndices{};
    bool isPresent{true};
};
struct TriangleOnEdge { TriangleIndex triangleIndex{}; std::size_t oppositeVertexSlot{}; };
struct TrianglesOnEdge { std::array<TriangleOnEdge, 2> triangles{}; std::size_t triangleCount{}; };
struct TriangleMesh {
    std::vector<StoredTriangle> triangles;
    std::map<Edge, TrianglesOnEdge> trianglesByEdge;
    bool hasValidTopology{true};
};

} // namespace detail
namespace {

using detail::Edge;
using detail::Triangle;
using detail::TriangleIndex;
using detail::StoredTriangle;
using detail::TriangleOnEdge;
using detail::TrianglesOnEdge;
using detail::TriangleMesh;
// Store the smaller index first so undirected edges have one canonical key.
Edge MakeEdge(std::size_t firstVertexIndex, std::size_t secondVertexIndex) {
    return firstVertexIndex < secondVertexIndex ? Edge{firstVertexIndex, secondVertexIndex} : Edge{secondVertexIndex, firstVertexIndex};
}

bool AddTriangle(TriangleMesh& triangleMesh, Triangle vertexIndices) {
    for (std::size_t opposite = 0; opposite < 3; ++opposite) {
        const Edge edge = MakeEdge(vertexIndices[(opposite + 1) % 3], vertexIndices[(opposite + 2) % 3]);
        auto edgeIterator = triangleMesh.trianglesByEdge.find(edge);
        if (edgeIterator != triangleMesh.trianglesByEdge.end() && edgeIterator->second.triangleCount == 2) { triangleMesh.hasValidTopology = false; return false; }
    }
    const TriangleIndex triangleIndex = triangleMesh.triangles.size();
    triangleMesh.triangles.push_back({vertexIndices, {}, true});
    for (std::size_t opposite = 0; opposite < 3; ++opposite) {
        const Edge edge = MakeEdge(vertexIndices[(opposite + 1) % 3], vertexIndices[(opposite + 2) % 3]);
        auto& trianglesOnEdge = triangleMesh.trianglesByEdge[edge];
        if (trianglesOnEdge.triangleCount == 1) {
            const auto neighboringTriangle = trianglesOnEdge.triangles[0];
            triangleMesh.triangles[triangleIndex].neighborTriangleIndices[opposite] = neighboringTriangle.triangleIndex;
            triangleMesh.triangles[neighboringTriangle.triangleIndex].neighborTriangleIndices[neighboringTriangle.oppositeVertexSlot] = triangleIndex;
        }
        trianglesOnEdge.triangles[trianglesOnEdge.triangleCount++] = {triangleIndex, opposite};
    }
    return true;
}

void RemoveTriangle(TriangleMesh& triangleMesh, TriangleIndex triangleIndex) {
    StoredTriangle& triangle = triangleMesh.triangles[triangleIndex];
    if (!triangle.isPresent) return;
    for (std::size_t opposite = 0; opposite < 3; ++opposite) {
        const Edge edge = MakeEdge(triangle.vertexIndices[(opposite + 1) % 3], triangle.vertexIndices[(opposite + 2) % 3]);
        auto edgeIterator = triangleMesh.trianglesByEdge.find(edge);
        if (edgeIterator == triangleMesh.trianglesByEdge.end()) { triangleMesh.hasValidTopology = false; continue; }
        auto& trianglesOnEdge = edgeIterator->second;
        for (std::size_t i = 0; i < trianglesOnEdge.triangleCount; ++i) if (trianglesOnEdge.triangles[i].triangleIndex == triangleIndex) {
            trianglesOnEdge.triangles[i] = trianglesOnEdge.triangles[trianglesOnEdge.triangleCount - 1];
            --trianglesOnEdge.triangleCount;
            if (trianglesOnEdge.triangleCount == 0) triangleMesh.trianglesByEdge.erase(edgeIterator);
            else {
                const auto survivor = trianglesOnEdge.triangles[0];
                triangleMesh.triangles[survivor.triangleIndex].neighborTriangleIndices[survivor.oppositeVertexSlot].reset();
            }
            break;
        }
    }
    triangle.isPresent = false;
}

bool HasEdge(const TriangleMesh& triangleMesh, const Edge& edge) { return triangleMesh.trianglesByEdge.contains(edge); }

// Set up the exact geometry checks once. The predicates need this initialization before use.
void InitializePredicates() {
    static const bool initialized = [] { mp_float::exactinit(); return true; }();
    (void)initialized;
}

// Return the signed side of C relative to AB. Other functions use the sign to test turns and crossings.
double Orient(const Vector2d& firstPoint, const Vector2d& secondPoint, const Vector2d& thirdPoint) {
    InitializePredicates();
    const double firstCoordinates[]{firstPoint.x, firstPoint.y}, secondCoordinates[]{secondPoint.x, secondPoint.y}, thirdCoordinates[]{thirdPoint.x, thirdPoint.y};
    return geompred::orient2d(firstCoordinates, secondCoordinates, thirdCoordinates);
}

// Return the signed circle test for D against ABC. The triangulation uses it to choose triangles to replace.
double InBall(const Vector2d& firstTrianglePoint, const Vector2d& secondTrianglePoint,
    const Vector2d& thirdTrianglePoint, const Vector2d& testPoint) {
    InitializePredicates();
    const double firstCoordinates[]{firstTrianglePoint.x, firstTrianglePoint.y};
    const double secondCoordinates[]{secondTrianglePoint.x, secondTrianglePoint.y};
    const double thirdCoordinates[]{thirdTrianglePoint.x, thirdTrianglePoint.y};
    const double testCoordinates[]{testPoint.x, testPoint.y};
    return geompred::inball2d(firstCoordinates, secondCoordinates, thirdCoordinates, testCoordinates);
}

// Reverse two loopVertexIndices when needed so later geometry checks receive a counterclockwise triangle.
Triangle Ccw(Triangle t, const std::vector<Vector2d>& points) {
    if (Orient(points[t[0]], points[t[1]], points[t[2]]) < 0.0) std::swap(t[1], t[2]);
    return t;
}

// Store a loop using loopVertexIndices into the shared point list, with its border or obstacle role.
struct LoopIndices { std::vector<std::size_t> vertices; bool obstacle{}; };

// Keep the output vertices, their positions, and the selected loops together during generation.
struct InputGeometry {
    ConstrainedMesh mesh;
    std::vector<Vector2d> points;
    std::vector<LoopIndices> loops;
};

// Test the loop edges with a ray from the point. This lets generation keep only triangles inside the border.
bool PointInLoop(const Vector2d& point, const LoopIndices& loop, const std::vector<Vector2d>& points) {
    // Begin: count horizontal-ray crossings. Each crossingEdges toggles inside/outside; this classifies triangle centers.
    // Count crossings of a horizontal ray; an odd count means the point is inside the loop.
    bool inside = false;
    for (std::size_t i = 0, j = loop.vertices.size() - 1; i < loop.vertices.size(); j = i++) {
        const Vector2d& a = points[loop.vertices[i]];
        const Vector2d& b = points[loop.vertices[j]];
        if ((a.y > point.y) != (b.y > point.y) &&
            point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    // End: the odd/even crossingEdges result is the point's loop membership.
    return inside;
}

// Compare which side each segment's endpoints lie on. Opposite sides on both segments mean they cross inside.
bool ProperlyIntersects(const Vector2d& a, const Vector2d& b, const Vector2d& c, const Vector2d& d) {
    const double ab_c = Orient(a, b, c), ab_d = Orient(a, b, d);
    const double cd_a = Orient(c, d, a), cd_b = Orient(c, d, b);
    return ((ab_c > 0.0 && ab_d < 0.0) || (ab_c < 0.0 && ab_d > 0.0)) &&
        ((cd_a > 0.0 && cd_b < 0.0) || (cd_a < 0.0 && cd_b > 0.0));
}

// Find the two triangles sharing an edge and replace their shared edge with the other diagonal.
// Reject cavityBoundaryEdges edges, non-convex pairs, and zero-area results so the mesh stays valid.
bool FlipEdge(TriangleMesh& triangleMesh, const Edge& edge, const std::vector<Vector2d>& points) {
    // Step 1: adjacency gives both incident faces and opposite vertices directly.
    auto edgeIterator = triangleMesh.trianglesByEdge.find(edge);
    if (edgeIterator == triangleMesh.trianglesByEdge.end() || edgeIterator->second.triangleCount != 2) return false;
    const auto adjacentTriangles = edgeIterator->second.triangles;
    const TriangleIndex firstTriangleIndex = adjacentTriangles[0].triangleIndex;
    const TriangleIndex secondTriangleIndex = adjacentTriangles[1].triangleIndex;
    const std::size_t c = triangleMesh.triangles[firstTriangleIndex].vertexIndices[adjacentTriangles[0].oppositeVertexSlot];
    const std::size_t d = triangleMesh.triangles[secondTriangleIndex].vertexIndices[adjacentTriangles[1].oppositeVertexSlot];
    // End: the candidate edge has two adjacent triangles to check.

    // Begin: verify the replacement diagonal makes two non-flat triangles, then replace the old faces.
    // This keeps the mesh connected and avoids introducing zero-area faces.
    const std::size_t a = edge[0], b = edge[1];
    if (c == d || !ProperlyIntersects(points[a], points[b], points[c], points[d])) return false;
    Triangle first = Ccw({c, d, a}, points), second = Ccw({d, c, b}, points);
    if (Orient(points[first[0]], points[first[1]], points[first[2]]) == 0.0 ||
        Orient(points[second[0]], points[second[1]], points[second[2]]) == 0.0) return false;
    // Step 2: remove the old pair, then register the replacement pair and all local links.
    RemoveTriangle(triangleMesh, firstTriangleIndex); RemoveTriangle(triangleMesh, secondTriangleIndex);
    if (!AddTriangle(triangleMesh, first) || !AddTriangle(triangleMesh, second)) { triangleMesh.hasValidTopology = false; return false; }
    // End: both faces now use the replacement diagonal.
    return true;
}

// Add points one at a time, replacing triangles whose circles contain each point with new triangles.
// A large temporary triangle starts the process; it is removed before returning the mesh.
std::vector<Triangle> BowyerWatson(const std::vector<Vector2d>& input, std::vector<Vector2d>& points) {
    // Find the input bounds and create a temporary triangle around them so every point starts inside a face.
    points = input;
    double minimumX = points[0].x, maximumX = minimumX, minimumY = points[0].y, maximumY = minimumY;
    for (const Vector2d& p : points) {
        minimumX = std::min(minimumX, p.x); maximumX = std::max(maximumX, p.x);
        minimumY = std::min(minimumY, p.y); maximumY = std::max(maximumY, p.y);
    }
    const double boundingBoxSpan = std::max(maximumX - minimumX, maximumY - minimumY);
    if (!std::isfinite(boundingBoxSpan) || boundingBoxSpan <= 0.0) return {};
    const double centerX = (minimumX + maximumX) * 0.5, centerY = (minimumY + maximumY) * 0.5;
    const std::size_t n = points.size();
    points.emplace_back(centerX - 32.0 * boundingBoxSpan, centerY - 16.0 * boundingBoxSpan);
    points.emplace_back(centerX + 32.0 * boundingBoxSpan, centerY - 16.0 * boundingBoxSpan);
    points.emplace_back(centerX, centerY + 32.0 * boundingBoxSpan);
    TriangleMesh triangleMesh;
    AddTriangle(triangleMesh, Ccw({n, n + 1, n + 2}, points));

    // Begin: insert each input point by removing affected triangles and filling their exposed outline.
    // This updates the mesh incrementally as each point is added.
    for (std::size_t pointIndex = 0; pointIndex < n; ++pointIndex) {
        std::vector<bool> badTriangles(triangleMesh.triangles.size(), false);
        std::map<Edge, int> cavityBoundaryEdges;
        bool foundCavityTriangle = false;
        for (std::size_t triangleIndex = 0; triangleIndex < triangleMesh.triangles.size(); ++triangleIndex) {
            if (!triangleMesh.triangles[triangleIndex].isPresent) continue;
            const Triangle& t = triangleMesh.triangles[triangleIndex].vertexIndices;
            if (InBall(points[t[0]], points[t[1]], points[t[2]], points[pointIndex]) > 0.0) {
                badTriangles[triangleIndex] = true; foundCavityTriangle = true;
                for (int edgeIndex = 0; edgeIndex < 3; ++edgeIndex) ++cavityBoundaryEdges[MakeEdge(t[edgeIndex], t[(edgeIndex + 1) % 3])];
            }
        }
        if (!foundCavityTriangle) continue;
        // Step: retain the existing global cavity scan, but update topology through adjacency helpers.
        for (std::size_t triangleIndex = 0; triangleIndex < badTriangles.size(); ++triangleIndex) if (badTriangles[triangleIndex]) RemoveTriangle(triangleMesh, triangleIndex);
        for (const auto& [edge, count] : cavityBoundaryEdges) if (count == 1) {
            Triangle t = Ccw({edge[0], edge[1], pointIndex}, points);
            if (Orient(points[t[0]], points[t[1]], points[t[2]]) != 0.0) AddTriangle(triangleMesh, t);
        }
        if (!triangleMesh.hasValidTopology) break;
    }
    // End: all input points have been added to the temporary triangulation.

    // Remove faces that use temporary vertices and restore the point list to only the input vertices.
    std::vector<Triangle> triangles;
    for (const StoredTriangle& triangle : triangleMesh.triangles) if (triangle.isPresent && triangle.vertexIndices[0] < n && triangle.vertexIndices[1] < n && triangle.vertexIndices[2] < n) triangles.push_back(triangle.vertexIndices);
    points.resize(n);
    return triangleMesh.hasValidTopology ? triangles : std::vector<Triangle>{};
}

// Add a required loop edge by repeatedly flipping mesh edges that cross it.
// Stop when the edge exists or no allowed flip can make progress.
bool RecoverConstraint(const Edge& constraint, TriangleMesh& triangleMesh,
    const std::vector<Vector2d>& points, std::set<Edge>& fixedConstraintEdges) {
    const std::size_t iterationLimit = triangleMesh.trianglesByEdge.size() * triangleMesh.trianglesByEdge.size() + 32;
    // Begin: flip crossingEdges mesh edges until the required loop edge appears or no flip can help.
    // Mark recovered edges fixedConstraintEdges so later steps do not remove the border or obstacle outline.
    for (std::size_t iteration = 0; iteration < iterationLimit; ++iteration) {
        if (HasEdge(triangleMesh, constraint)) { fixedConstraintEdges.insert(constraint); return true; }
        std::vector<Edge> crossingEdges;
        for (const auto& [edge, trianglesOnEdge] : triangleMesh.trianglesByEdge) {
            (void)trianglesOnEdge;
            if (edge != constraint && ProperlyIntersects(points[constraint[0]], points[constraint[1]],
                points[edge[0]], points[edge[1]])) crossingEdges.push_back(edge);
        }
        std::sort(crossingEdges.begin(), crossingEdges.end());
        crossingEdges.erase(std::unique(crossingEdges.begin(), crossingEdges.end()), crossingEdges.end());
        bool flipped = false;
        for (const Edge& edge : crossingEdges) if (!fixedConstraintEdges.contains(edge) && FlipEdge(triangleMesh, edge, points)) {
            flipped = true; break;
        }
        if (!flipped) return false;
    }
    // End: reaching the iterationLimit means recovery did not make enough progress.
    return false;
}

// Check shared, non-constraint edges and flip ones that violate the circle rule.
// Repeat until no edge needs changing or the safety iterationLimit is reached.
bool Legalize(TriangleMesh& triangleMesh, const std::vector<Vector2d>& points,
    const std::set<Edge>& fixedConstraintEdges) {
    const std::size_t iterationLimit = triangleMesh.trianglesByEdge.size() * triangleMesh.trianglesByEdge.size() * 8 + 64;
    // Seed the worklist once; each successful flip adds only its local edges.
    std::deque<Edge> edgesToCheck;
    for (const auto& [edge, trianglesOnEdge] : triangleMesh.trianglesByEdge) { (void)trianglesOnEdge; edgesToCheck.push_back(edge); }
    std::size_t checkedEdgeCount = 0;
    while (!edgesToCheck.empty() && checkedEdgeCount++ < iterationLimit) {
        const Edge edge = edgesToCheck.front(); edgesToCheck.pop_front();
        auto it = triangleMesh.trianglesByEdge.find(edge);
        if (it == triangleMesh.trianglesByEdge.end() || it->second.triangleCount != 2 || fixedConstraintEdges.contains(edge)) continue;
        const auto adjacentTriangles = it->second.triangles;
        const std::size_t c = triangleMesh.triangles[adjacentTriangles[0].triangleIndex].vertexIndices[adjacentTriangles[0].oppositeVertexSlot];
        const std::size_t d = triangleMesh.triangles[adjacentTriangles[1].triangleIndex].vertexIndices[adjacentTriangles[1].oppositeVertexSlot];
        if (c == d || !ProperlyIntersects(points[edge[0]], points[edge[1]], points[c], points[d])) continue;
        Triangle t = Ccw({edge[0], edge[1], c}, points);
        if (InBall(points[t[0]], points[t[1]], points[t[2]], points[d]) <= 0.0) continue;
        const std::size_t old_triangle_count = triangleMesh.triangles.size();
        if (!FlipEdge(triangleMesh, edge, points)) continue;
        // Recheck the changed faces' three edges; these include every edge whose opposite face changed.
        for (std::size_t triangleIndex = old_triangle_count; triangleIndex < triangleMesh.triangles.size(); ++triangleIndex) {
            const StoredTriangle& triangle = triangleMesh.triangles[triangleIndex];
            for (std::size_t opposite = 0; opposite < 3; ++opposite)
                edgesToCheck.push_back(MakeEdge(triangle.vertexIndices[(opposite + 1) % 3], triangle.vertexIndices[(opposite + 2) % 3]));
        }
    }
    if (edgesToCheck.empty()) return triangleMesh.hasValidTopology;
    // End: the safety iterationLimit was reached before the mesh settled.
    return false;
}

// Copy one selected loop into the shared input lists and preserve its source vertex IDs.
// The index map avoids adding a shared model vertex twice; false means the loop data is incomplete.
bool AddLoop(const GeometryModel& model, const GeometryLoop& loop, bool obstacle,
    InputGeometry& input, std::unordered_map<VertexId, std::size_t>& inputVertexIndexById) {
    // Begin: resolve each loop vertex and assign a shared index for the mesh and triangle routines.
    // Invalid or non-finite vertices stop collection because later geometry checks need valid coordinates.
    LoopIndices loopVertexIndices;
    loopVertexIndices.obstacle = obstacle;
    for (VertexId id : loop.vertices) {
        const Vertex* vertex = model.FindVertex(id);
        if (!vertex || !std::isfinite(vertex->position.x) || !std::isfinite(vertex->position.y)) return false;
        auto [it, wasInserted] = inputVertexIndexById.emplace(id, input.mesh.vertices.size());
        if (wasInserted) {
            input.mesh.vertices.push_back({id, vertex->position});
            input.points.push_back(vertex->position);
        }
        loopVertexIndices.vertices.push_back(it->second);
    }
    if (loopVertexIndices.vertices.size() < 3) return false;
    input.loops.push_back(std::move(loopVertexIndices));
    // End: the complete loop is now stored in the shared input geometry.
    return true;
}

// Select the active border and eligible obstacles, then copy their points into the triangulation input.
// Return a specific message if validation and model data do not agree.
bool CollectInputLoops(const GeometryModel& model, const GeometryValidation& validation,
    InputGeometry& input, std::string& error) {
    if (!validation.active_border_index) {
        error = "No valid active border.";
        return false;
    }

    const std::size_t border_index = *validation.active_border_index;
    if (border_index >= model.Loops().size() || border_index >= validation.loops.size() ||
        !validation.loops[border_index].active_border) {
        error = "Active border input is inconsistent.";
        return false;
    }
    // Begin: add the active border first because later code uses it as the domain cavityBoundaryEdges.
    std::unordered_map<VertexId, std::size_t> inputVertexIndexById;
    if (!AddLoop(model, model.Loops()[border_index], false, input, inputVertexIndexById)) {
        error = "Active border input is inconsistent.";
        return false;
    }
    // End: the border is the first loop in the input list.

    // Include only validated obstacle loops; other loops must not affect the generated mesh.
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        if (i == border_index || i >= validation.loops.size() || !validation.loops[i].eligible ||
            model.Loops()[i].type != EdgeType::Obstacle) continue;
        if (!AddLoop(model, model.Loops()[i], true, input, inputVertexIndexById)) {
            error = "Obstacle input is inconsistent.";
            return false;
        }
    }
    return true;
}

// Make one required mesh edge for each consecutive pair of vertices in each selected loop.
// These edges keep the generated triangles aligned with the border and obstacle outlines.
bool BuildConstraints(const InputGeometry& input, std::set<Edge>& constraints, std::string& error) {
    // Constraint splitting remains disabled for now. Each original loop edge is usedVertices as-is.
    // Begin: connect each loop vertex to the next, including the closing edge.
    // These are the exact edges the triangulation must preserve as boundaries.
    for (const LoopIndices& loop : input.loops) for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
        const std::size_t startVertexIndex = loop.vertices[i], endVertexIndex = loop.vertices[(i + 1) % loop.vertices.size()];
        if (startVertexIndex == endVertexIndex) {
            error = "Coincident constraint endpoints.";
            return false;
        }
        constraints.insert(MakeEdge(startVertexIndex, endVertexIndex));
    }
    // End: all original loop edges are recorded as required constraints.
    return true;
}

// Find duplicate positions belonging to different input vertices, which the triangulator cannot handle.
bool HasCoincidentPoints(const std::vector<Vector2d>& points) {
    // Compare each pair once so the caller can report invalid input before building triangles.
    // Begin: scan every unique pair for identical coordinates.
    for (std::size_t i = 0; i < points.size(); ++i)
        for (std::size_t j = i + 1; j < points.size(); ++j)
            if (points[i] == points[j]) return true;
    // End: no duplicate coordinate pair was foundCavityTriangle.
    return false;
}

// Create the initialTriangles triangle mesh, add all required loop edges, then improve its unconstrained edges.
// Stop at the first failed stage and return its message to the caller.
bool BuildConstrainedTriangulation(const InputGeometry& input, const std::set<Edge>& constraints,
    std::vector<Triangle>& triangles, std::string& error) {
    // Begin: build triangles from the points, then recover and protect each required loop edge.
    // These stages create the constrained mesh that respects the selected outlines.
    std::vector<Vector2d> points;
    std::vector<Triangle> initialTriangles = BowyerWatson(input.points, points);
    if (initialTriangles.empty()) {
        error = "Could not triangulate input vertices.";
        return false;
    }
    TriangleMesh triangleMesh;
    for (const Triangle& triangle : initialTriangles) if (!AddTriangle(triangleMesh, triangle)) {
        error = "Could not build triangle adjacency.";
        return false;
    }

    std::set<Edge> fixedConstraintEdges;
    for (const Edge& edge : constraints) if (!RecoverConstraint(edge, triangleMesh, points, fixedConstraintEdges)) {
        error = "Could not recover every cavityBoundaryEdges constraint.";
        return false;
    }
    if (!Legalize(triangleMesh, points, fixedConstraintEdges)) {
        error = "Constrained edge legalization did not converge.";
        return false;
    }
    triangles.clear();
    for (const StoredTriangle& triangle : triangleMesh.triangles)
        if (triangle.isPresent) triangles.push_back(triangle.vertexIndices);
    // End: the mesh contains all constraints and its remaining edges have been improved.
    return true;
}

// Keep triangles inside the border and label those inside an obstacle as blocked.
// Triangle centers provide a simple point to test against each selected loop.
void AddTrianglesInsideLoops(const InputGeometry& input, const std::vector<Triangle>& triangles,
    ConstrainedMesh& mesh) {
    // Begin: classify each triangle by its triangleCenter, keeping only those inside the border.
    // Obstacle membership changes the triangleRegion label so navigation can treat those triangles as blocked.
    for (const Triangle& triangle : triangles) {
        const Vector2d triangleCenter = (input.points[triangle[0]] + input.points[triangle[1]] +
            input.points[triangle[2]]) / 3.0;
        if (!PointInLoop(triangleCenter, input.loops.front(), input.points)) continue;
        TriangleRegion triangleRegion = TriangleRegion::Traversable;
        bool inside_hole = false;
        for (std::size_t loop_index = 1; loop_index < input.loops.size(); ++loop_index)
            if (PointInLoop(triangleCenter, input.loops[loop_index], input.points)) {
                inside_hole = true;
                break;
            }
        if (inside_hole) continue;
        mesh.triangles.push_back({triangle, triangleRegion});
    }
    // End: the output mesh now contains only in-domain triangles with triangleRegion labels.
}

// Check that clipping did not leave any selected input vertex without a triangle.
bool UsesEveryInputVertex(const ConstrainedMesh& mesh) {
    // Mark each referenced vertex, then require a non-empty mesh with no unused vertices.
    // Begin: record vertex use from all retained triangles.
    std::vector<bool> usedVertices(mesh.vertices.size(), false);
    for (const MeshTriangle& triangle : mesh.triangles)
        for (std::size_t vertex : triangle.vertices) usedVertices[vertex] = true;
    // End: accept only a non-empty mesh that uses every selected input vertex.
    return !mesh.triangles.empty() && std::find(usedVertices.begin(), usedVertices.end(), false) == usedVertices.end();
}

} // namespace

// Build a mesh from the active border and eligible obstacle loops.
// Input loops become constraints, triangles are selected by location, and failures return a message.
GenerationResult ConstrainedDelaunay::Generate(const GeometryModel& model,
    const GeometryValidation& validation, double agent_radius) const {
    if (!std::isfinite(agent_radius) || agent_radius < 0.0)
        return {GenerationStatus::Failed, {}, "Agent radius must be a finite, nonnegative world-space value."};
    if (!validation.active_border_index)
        return {GenerationStatus::NoActiveBorder, {}, "No valid active border."};

    using namespace Clipper2Lib;
    // A larger arc tolerance produces fewer vertices along the approximated circular joins.
    constexpr double round_join_tolerance_fraction = 0.20;
    const double round_join_tolerance = agent_radius * round_join_tolerance_fraction;
    const std::size_t border_index = *validation.active_border_index;
    if (border_index >= model.Loops().size() || border_index >= validation.loops.size() ||
        !validation.loops[border_index].active_border)
        return {GenerationStatus::Failed, {}, "Active border input is inconsistent."};

    auto source_path = [&](const GeometryLoop& loop, PathD& path) {
        for (VertexId id : loop.vertices) {
            const Vertex* vertex = model.FindVertex(id);
            if (!vertex || !std::isfinite(vertex->position.x) || !std::isfinite(vertex->position.y)) return false;
            path.emplace_back(vertex->position.x, vertex->position.y);
        }
        return path.size() >= 3;
    };
    PathsD border_paths;
    PathD border;
    if (!source_path(model.Loops()[border_index], border))
        return {GenerationStatus::Failed, {}, "Active border input is inconsistent."};
    border_paths.push_back(std::move(border));
    PathsD inset_border = InflatePaths(border_paths, -agent_radius, JoinType::Round, EndType::Polygon,
        2.0, 8, round_join_tolerance);
    if (inset_border.empty())
        return {GenerationStatus::Failed, {}, "Agent radius removes the walkable border."};

    PathsD obstacles;
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        if (i == border_index || i >= validation.loops.size() || !validation.loops[i].eligible ||
            model.Loops()[i].type != EdgeType::Obstacle) continue;
        PathD path;
        if (!source_path(model.Loops()[i], path))
            return {GenerationStatus::Failed, {}, "Obstacle input is inconsistent."};
        PathsD expanded = InflatePaths(PathsD{path}, agent_radius, JoinType::Round, EndType::Polygon,
            2.0, 8, round_join_tolerance);
        obstacles.insert(obstacles.end(), expanded.begin(), expanded.end());
    }
    if (!obstacles.empty()) obstacles = Union(obstacles, FillRule::NonZero, 8);
    PathsD free_contours = Difference(inset_border, obstacles, FillRule::NonZero, 8);
    if (free_contours.empty())
        return {GenerationStatus::Failed, {}, "Agent radius leaves no walkable area."};
    auto contains_path = [](const PathD& path, PointD point) {
        bool inside = false;
        for (std::size_t i = 0, j = path.size() - 1; i < path.size(); j = i++) {
            const auto& a = path[i]; const auto& b = path[j];
            if ((a.y > point.y) != (b.y > point.y) &&
                point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
        }
        return inside;
    };
    ConstrainedMesh combined;
    std::size_t component_count = 0;
    for (const PathD& outer : free_contours) {
        if (Area(outer) <= 0.0) continue; // Clipper orients holes opposite to outer contours.
        InputGeometry input;
        auto add_path = [&](const PathD& path, bool obstacle) {
            LoopIndices loop;
            loop.obstacle = obstacle;
            for (const PointD& point : path) {
                const Vector2d position{point.x, point.y};
                std::optional<VertexId> source_id;
                for (const Vertex& vertex : model.Vertices())
                    if (vertex.position == position) { source_id = vertex.id; break; }
                const std::size_t index = input.points.size();
                input.points.push_back(position);
                input.mesh.vertices.push_back({source_id, position});
                loop.vertices.push_back(index);
            }
            if (loop.vertices.size() >= 3) input.loops.push_back(std::move(loop));
        };
        add_path(outer, false);
        for (const PathD& contour : free_contours)
            if (Area(contour) < 0.0 && !contour.empty() && contains_path(outer, contour.front()))
                add_path(contour, true);
        if (input.loops.empty()) continue;

        std::string error;
        std::set<Edge> constraints;
        if (!BuildConstraints(input, constraints, error) || input.points.size() < 3 || HasCoincidentPoints(input.points))
            return {GenerationStatus::Failed, {}, "Offset contours contain invalid or coincident vertices."};
        std::vector<Triangle> triangles;
        if (!BuildConstrainedTriangulation(input, constraints, triangles, error))
            return {GenerationStatus::Failed, {}, "Could not triangulate offset contours: " + error};
        AddTrianglesInsideLoops(input, triangles, input.mesh);
        if (!UsesEveryInputVertex(input.mesh))
            return {GenerationStatus::Failed, {}, "An offset walkable component was not completely triangulated."};

        const std::size_t base = combined.vertices.size();
        combined.vertices.insert(combined.vertices.end(), input.mesh.vertices.begin(), input.mesh.vertices.end());
        for (MeshTriangle triangle : input.mesh.triangles) {
            for (std::size_t& vertex : triangle.vertices) vertex += base;
            combined.triangles.push_back(triangle);
        }
        for (const Edge& edge : constraints) combined.constraints.push_back({edge[0] + base, edge[1] + base});
        ++component_count;
    }

    if (component_count == 0 || combined.triangles.empty())
        return {GenerationStatus::Failed, {}, "Agent radius leaves no triangulatable walkable area."};
    return {GenerationStatus::Success, std::move(combined), "Navmesh baked successfully."};
}

} // namespace cdt
