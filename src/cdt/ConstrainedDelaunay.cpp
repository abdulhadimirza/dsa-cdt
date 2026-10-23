#include "cdt/ConstrainedDelaunay.hpp"
#include "cdt/WalkableRegionBuilder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <map>
#include <optional>
#include <set>

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

// Reverse two vertex indices when needed so later geometry checks receive a counterclockwise triangle.
Triangle Ccw(Triangle t, const std::vector<Vector2d>& points) {
    if (Orient(points[t[0]], points[t[1]], points[t[2]]) < 0.0) std::swap(t[1], t[2]);
    return t;
}

// Store one contour using indices into the shared point list.
struct ContourIndices { std::vector<std::size_t> vertices; };

// Keep the mesh vertices, point positions, and region contours together during generation.
struct InputGeometry {
    ConstrainedMesh mesh;
    std::vector<Vector2d> points;
    std::vector<ContourIndices> contours;
};

// Test contour membership with a ray so generation can keep only triangles inside the border.
bool PointInLoop(const Vector2d& point, const ContourIndices& contour, const std::vector<Vector2d>& points) {
    // Count horizontal-ray crossings; an odd count means the point is inside the contour.
    bool inside = false;
    for (std::size_t i = 0, j = contour.vertices.size() - 1; i < contour.vertices.size(); j = i++) {
        const Vector2d& a = points[contour.vertices[i]];
        const Vector2d& b = points[contour.vertices[j]];
        if ((a.y > point.y) != (b.y > point.y) &&
            point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
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
// Reject constraint edges, non-convex pairs, and zero-area results so the mesh stays valid.
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
    // Add a temporary enclosing triangle so every input point begins inside the triangulation.
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

    // Point insertion: remove triangles whose circumcircles contain the point, then fill the cavity
    // from edges that occur only once on the removed triangles' boundary.
    for (std::size_t pointIndex = 0; pointIndex < n; ++pointIndex) {
        std::vector<bool> trianglesToRemove(triangleMesh.triangles.size(), false);
        std::map<Edge, int> boundaryEdgeCounts;
        bool foundCavityTriangle = false;
        for (std::size_t triangleIndex = 0; triangleIndex < triangleMesh.triangles.size(); ++triangleIndex) {
            if (!triangleMesh.triangles[triangleIndex].isPresent) continue;
            const Triangle& t = triangleMesh.triangles[triangleIndex].vertexIndices;
            if (InBall(points[t[0]], points[t[1]], points[t[2]], points[pointIndex]) > 0.0) {
                trianglesToRemove[triangleIndex] = true; foundCavityTriangle = true;
                for (int edgeIndex = 0; edgeIndex < 3; ++edgeIndex) ++boundaryEdgeCounts[MakeEdge(t[edgeIndex], t[(edgeIndex + 1) % 3])];
            }
        }
        if (!foundCavityTriangle) continue;
        for (std::size_t triangleIndex = 0; triangleIndex < trianglesToRemove.size(); ++triangleIndex)
            if (trianglesToRemove[triangleIndex]) RemoveTriangle(triangleMesh, triangleIndex);
        for (const auto& [edge, count] : boundaryEdgeCounts) if (count == 1) {
            Triangle t = Ccw({edge[0], edge[1], pointIndex}, points);
            if (Orient(points[t[0]], points[t[1]], points[t[2]]) != 0.0) AddTriangle(triangleMesh, t);
        }
        if (!triangleMesh.hasValidTopology) break;
    }
    // Remove faces that use temporary vertices before returning the completed triangulation.
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
    // Constraint recovery: flip crossing mesh edges until the required edge exists, then protect it
    // from legalization so the input contour remains part of the mesh boundary.
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
    return false;
}

// Check shared, non-constraint edges and flip ones that violate the circle rule.
// Repeat until no edge needs changing or the safety iterationLimit is reached.
bool Legalize(TriangleMesh& triangleMesh, const std::vector<Vector2d>& points,
    const std::set<Edge>& fixedConstraintEdges) {
    const std::size_t iterationLimit = triangleMesh.trianglesByEdge.size() * triangleMesh.trianglesByEdge.size() * 8 + 64;
    // Legalization: check shared, unconstrained edges and flip those that violate the Delaunay circle rule.
    // Changed faces add their edges back to the worklist; the limit prevents an endless loop.
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

// Make one required mesh edge for each consecutive pair of vertices in each selected contour.
// These edges keep the generated triangles aligned with the border and obstacle outlines.
bool BuildConstraints(const InputGeometry& input, std::set<Edge>& constraints, std::string& error) {
    // Record each contour edge, including the edge from its last vertex back to its first.
    for (const ContourIndices& contour : input.contours) for (std::size_t i = 0; i < contour.vertices.size(); ++i) {
        const std::size_t startVertexIndex = contour.vertices[i], endVertexIndex = contour.vertices[(i + 1) % contour.vertices.size()];
        if (startVertexIndex == endVertexIndex) {
            error = "Coincident constraint endpoints.";
            return false;
        }
        constraints.insert(MakeEdge(startVertexIndex, endVertexIndex));
    }
    return true;
}

// Find duplicate positions belonging to different input vertices, which the triangulator cannot handle.
bool HasCoincidentPoints(const std::vector<Vector2d>& points) {
    // Compare each pair once because the triangulator cannot use coincident input points.
    for (std::size_t i = 0; i < points.size(); ++i)
        for (std::size_t j = i + 1; j < points.size(); ++j)
            if (points[i] == points[j]) return true;
    return false;
}

// Create the initial triangle mesh, recover all required contour edges, then legalize its free edges.
// Stop at the first failed stage and return its message to the caller.
bool BuildConstrainedTriangulation(const InputGeometry& input, const std::set<Edge>& constraints,
    std::vector<Triangle>& triangles, std::string& error) {
    // Build the initial triangulation, recover constraints, and legalize unconstrained edges in order.
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
        error = "Could not recover every contour constraint.";
        return false;
    }
    if (!Legalize(triangleMesh, points, fixedConstraintEdges)) {
        error = "Constrained edge legalization did not converge.";
        return false;
    }
    triangles.clear();
    for (const StoredTriangle& triangle : triangleMesh.triangles)
        if (triangle.isPresent) triangles.push_back(triangle.vertexIndices);
    return true;
}

// Keep triangles inside the outer contour and exclude triangles inside holes.
// Triangle centers provide a simple point to test against each contour.
void KeepTrianglesInsideRegion(const InputGeometry& input, const std::vector<Triangle>& triangles,
    ConstrainedMesh& mesh) {
    // Output filtering: keep triangle centers inside the outer contour and discard centers inside holes.
    for (const Triangle& triangle : triangles) {
        const Vector2d triangleCenter = (input.points[triangle[0]] + input.points[triangle[1]] +
            input.points[triangle[2]]) / 3.0;
        if (!PointInLoop(triangleCenter, input.contours.front(), input.points)) continue;
        const bool insideHole = [&] {
            for (std::size_t contourIndex = 1; contourIndex < input.contours.size(); ++contourIndex)
                if (PointInLoop(triangleCenter, input.contours[contourIndex], input.points)) return true;
            return false;
        }();
        if (insideHole) continue;
        mesh.triangles.push_back({triangle, TriangleRegion::Traversable});
    }
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

void AppendComponentMesh(const InputGeometry& input, const std::set<Edge>& constraints,
    ConstrainedMesh& combinedMesh) {
    const std::size_t baseVertexIndex = combinedMesh.vertices.size();
    combinedMesh.vertices.insert(combinedMesh.vertices.end(), input.mesh.vertices.begin(), input.mesh.vertices.end());
    for (MeshTriangle triangle : input.mesh.triangles) {
        for (std::size_t& vertexIndex : triangle.vertices) vertexIndex += baseVertexIndex;
        combinedMesh.triangles.push_back(triangle);
    }
    for (const Edge& edge : constraints)
        combinedMesh.constraints.push_back({edge[0] + baseVertexIndex, edge[1] + baseVertexIndex});
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

    // Build walkable regions separately, then triangulate and merge each connected component.
    std::vector<detail::WalkableRegion> walkableRegions;
    std::string error;
    if (!detail::BuildWalkableRegions(model, validation, agent_radius, walkableRegions, error))
        return {GenerationStatus::Failed, {}, error};

    ConstrainedMesh combined;
    std::size_t componentCount = 0;
    for (const detail::WalkableRegion& region : walkableRegions) {
        InputGeometry input;
        auto addContour = [&](const std::vector<Vector2d>& contour) {
            ContourIndices contourIndices;
            contourIndices.vertices.reserve(contour.size());
            for (const Vector2d& position : contour) {
                std::optional<VertexId> sourceId;
                for (const Vertex& vertex : model.Vertices())
                    if (vertex.position == position) { sourceId = vertex.id; break; }
                const std::size_t index = input.points.size();
                input.points.push_back(position);
                input.mesh.vertices.push_back({sourceId, position});
                contourIndices.vertices.push_back(index);
            }
            if (contourIndices.vertices.size() >= 3) input.contours.push_back(std::move(contourIndices));
        };
        addContour(region.outerContour);
        for (const std::vector<Vector2d>& holeContour : region.holeContours) addContour(holeContour);
        if (input.contours.empty()) continue;

        std::set<Edge> constraints;
        if (!BuildConstraints(input, constraints, error) || input.points.size() < 3 || HasCoincidentPoints(input.points))
            return {GenerationStatus::Failed, {}, "Offset contours contain invalid or coincident vertices."};
        std::vector<Triangle> triangles;
        if (!BuildConstrainedTriangulation(input, constraints, triangles, error))
            return {GenerationStatus::Failed, {}, "Could not triangulate offset contours: " + error};
        KeepTrianglesInsideRegion(input, triangles, input.mesh);
        if (!UsesEveryInputVertex(input.mesh))
            return {GenerationStatus::Failed, {}, "An offset walkable component was not completely triangulated."};

        AppendComponentMesh(input, constraints, combined);
        ++componentCount;
    }

    if (componentCount == 0 || combined.triangles.empty())
        return {GenerationStatus::Failed, {}, "Agent radius leaves no triangulatable walkable area."};
    return {GenerationStatus::Success, std::move(combined), "Navmesh baked successfully."};
}

} // namespace cdt
