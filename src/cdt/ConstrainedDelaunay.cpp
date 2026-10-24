#include "cdt/ConstrainedDelaunay.hpp"
#include "core/GeometryPredicates.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>


namespace cdt {

/// Create the public navmesh by moving the completed draft into its private storage.
/// @param draft Vertices, triangles, and constraints assembled by the triangulator.
/// @return No value; the new NavMesh owns the moved data after construction.
NavMesh::NavMesh(ConstrainedMesh&& draft)
    : vertices_(std::move(draft.vertices)), triangles_(std::move(draft.triangles)),
      constraints_(std::move(draft.constraints)) {}

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

using DiagnosticClock = std::chrono::steady_clock;

bool NavmeshDiagnosticsEnabled() {
    const char* value = std::getenv("CDT_NAVMESH_DIAGNOSTICS");
    return value && value[0] != '\0' && std::string_view(value) != "0";
}

double ElapsedMilliseconds(DiagnosticClock::time_point start) {
    return std::chrono::duration<double, std::milli>(DiagnosticClock::now() - start).count();
}

struct BowyerInsertionStats {
    std::size_t pointIndex{};
    Vector2d point{};
    std::size_t storedTriangleSlots{};
    std::size_t liveTrianglesScanned{};
    std::size_t cavityTriangles{};
    std::size_t boundaryEdges{};
    std::size_t trianglesAdded{};
    double elapsedMs{};
};

struct LegalizationStats {
    std::string stage;
    std::size_t initialQueueSize{};
    std::size_t checkedEdges{};
    std::size_t successfulFlips{};
    std::size_t maximumQueueSize{};
    std::size_t iterationLimit{};
    bool reachedLimit{};
    double elapsedMs{};
};

struct ConstraintRecoveryStats {
    Edge originalConstraint{};
    Edge segment{};
    Vector2d segmentStart{};
    Vector2d segmentEnd{};
    std::size_t constraintIndex{};
    std::size_t segmentIndex{};
    std::size_t segmentCount{};
    std::size_t pointChecks{};
    std::size_t iterations{};
    std::size_t iterationLimit{};
    std::size_t edgeEntriesScanned{};
    std::size_t intersectionTests{};
    std::size_t crossingEdgesSeen{};
    std::size_t flipAttempts{};
    std::size_t successfulFlips{};
    std::size_t progressFlips{};
    std::size_t nonProgressFlips{};
    std::size_t maximumNonProgressFlipStreak{};
    bool earlyCavityAttempted{};
    bool earlyCavityRecovered{};
    std::size_t cavityAttempts{};
    std::size_t cavityEdgeEntriesScanned{};
    std::size_t cavityCrossedEdges{};
    std::size_t cavityRemovedTriangles{};
    std::size_t cavityBoundaryVertices{};
    std::size_t cavityFillTriangles{};
    double cavityPathMs{};
    double cavityMs{};
    std::string cavityFailure;
    std::string outcome;
    double elapsedMs{};
};

struct CdtDiagnostics {
    explicit CdtDiagnostics(bool isEnabled) : enabled(isEnabled) {}

    bool enabled{};
    std::size_t hullVertices{};
    std::size_t interiorPointCandidates{};
    std::size_t pointsWithoutCavity{};
    std::size_t storedTriangleSlotsScanned{};
    std::size_t liveTrianglesScanned{};
    std::size_t circumcircleTests{};
    std::size_t cavityTrianglesRemoved{};
    std::size_t cavityBoundaryEdges{};
    std::size_t pointTrianglesAdded{};
    std::size_t maximumStoredSlotsAtScan{};
    std::size_t finalTriangleSlots{};
    std::size_t finalLiveTriangles{};
    std::size_t originalConstraintCount{};
    std::size_t constraintPointChecks{};
    std::size_t constraintSegments{};
    std::size_t recoveredSegments{};
    std::size_t constraintRecoveryIterationCount{};
    std::size_t constraintEdgeEntriesScanned{};
    std::size_t constraintIntersectionTests{};
    std::size_t constraintCrossingEdgesSeen{};
    std::size_t constraintFlipAttempts{};
    std::size_t constraintSuccessfulFlips{};
    std::size_t constraintProgressFlips{};
    std::size_t constraintNonProgressFlips{};
    std::size_t earlyCavityAttempts{};
    std::size_t earlyCavitySuccesses{};
    std::size_t cavityFallbackAttempts{};
    double bowyerWatsonMs{};
    double hullBuildMs{};
    double hullTriangulationMs{};
    double seedAdjacencyMs{};
    double constraintRecoveryMs{};
    double constrainedBuildMs{};
    std::string bowyerFailure;
    std::string constrainedBuildFailure;
    std::vector<BowyerInsertionStats> slowInsertions;
    std::vector<LegalizationStats> legalizations;
    std::vector<ConstraintRecoveryStats> recoverySegments;
};

/// Make an edge key that is the same whichever endpoint order the caller provides.
/// @param firstVertexIndex Index of one endpoint in the shared vertex list.
/// @param secondVertexIndex Index of the other endpoint.
/// @return The two indices in ascending order; the smaller one is always first.
// The key represents an undirected edge: A--B and B--A must compare equal.
Edge MakeEdge(std::size_t firstVertexIndex, std::size_t secondVertexIndex) {
    return firstVertexIndex < secondVertexIndex ? Edge{firstVertexIndex, secondVertexIndex} : Edge{secondVertexIndex, firstVertexIndex};
}

/// Add one triangle and update the edge table and neighbor links around it.
/// @param triangleMesh Mesh that will receive the triangle and its edge records.
/// @param vertexIndices The three vertex indices, in counterclockwise order.
/// @return True if the triangle was added; false if an edge would have more than two faces.
/// @note On a non-manifold edge, the mesh is marked as having invalid topology.
bool AddTriangle(TriangleMesh& triangleMesh, Triangle vertexIndices) {
    for (std::size_t opposite = 0; opposite < 3; ++opposite) {
        // Slot `opposite` refers to the edge across from that triangle vertex.
        const Edge edge = MakeEdge(vertexIndices[(opposite + 1) % 3], vertexIndices[(opposite + 2) % 3]);
        auto edgeIterator = triangleMesh.trianglesByEdge.find(edge);
        if (edgeIterator != triangleMesh.trianglesByEdge.end() && edgeIterator->second.triangleCount == 2) { triangleMesh.hasValidTopology = false; return false; }
    }
    const TriangleIndex triangleIndex = triangleMesh.triangles.size();
    triangleMesh.triangles.push_back({vertexIndices, {}, true});
    for (std::size_t opposite = 0; opposite < 3; ++opposite) {
        // Look up the same opposite-vertex edge again so this face can be linked to its neighbor.
        const Edge edge = MakeEdge(vertexIndices[(opposite + 1) % 3], vertexIndices[(opposite + 2) % 3]);
        auto& trianglesOnEdge = triangleMesh.trianglesByEdge[edge];
        if (trianglesOnEdge.triangleCount == 1) {
            // The edge already had one face, so this new face completes the pair.
            const auto neighboringTriangle = trianglesOnEdge.triangles[0];
            triangleMesh.triangles[triangleIndex].neighborTriangleIndices[opposite] = neighboringTriangle.triangleIndex;
            triangleMesh.triangles[neighboringTriangle.triangleIndex].neighborTriangleIndices[neighboringTriangle.oppositeVertexSlot] = triangleIndex;
        }
        trianglesOnEdge.triangles[trianglesOnEdge.triangleCount++] = {triangleIndex, opposite};
    }
    return true;
}

/// Remove a live triangle and update every edge and neighbor link that referred to it.
/// @param triangleMesh Mesh to change.
/// @param triangleIndex Position of the triangle in the mesh's triangle storage.
/// @return No value. A missing edge record marks the mesh topology invalid.
void RemoveTriangle(TriangleMesh& triangleMesh, TriangleIndex triangleIndex) {
    StoredTriangle& triangle = triangleMesh.triangles[triangleIndex];
    if (!triangle.isPresent) return;
    for (std::size_t opposite = 0; opposite < 3; ++opposite) {
        // As in AddTriangle, each slot describes the edge opposite the vertex at that slot.
        const Edge edge = MakeEdge(triangle.vertexIndices[(opposite + 1) % 3], triangle.vertexIndices[(opposite + 2) % 3]);
        auto edgeIterator = triangleMesh.trianglesByEdge.find(edge);
        if (edgeIterator == triangleMesh.trianglesByEdge.end()) { triangleMesh.hasValidTopology = false; continue; }
        auto& trianglesOnEdge = edgeIterator->second;
        for (std::size_t i = 0; i < trianglesOnEdge.triangleCount; ++i) if (trianglesOnEdge.triangles[i].triangleIndex == triangleIndex) {
            // Replace the removed entry with the last entry, then shorten the small fixed-size list.
            trianglesOnEdge.triangles[i] = trianglesOnEdge.triangles[trianglesOnEdge.triangleCount - 1];
            --trianglesOnEdge.triangleCount;
            if (trianglesOnEdge.triangleCount == 0) triangleMesh.trianglesByEdge.erase(edgeIterator);
            else {
                // A single face remains on this edge, so it is now a boundary edge with no neighbor.
                const auto survivor = trianglesOnEdge.triangles[0];
                triangleMesh.triangles[survivor.triangleIndex].neighborTriangleIndices[survivor.oppositeVertexSlot].reset();
            }
            break;
        }

    }
    triangle.isPresent = false;
}

/// Records where an edge was first seen while rebuilding final neighbor links.
/// `edgeSlot` is the triangle-neighbor slot across from that edge.
struct FirstEdgeUse {
    std::size_t triangleIndex{};
    std::size_t edgeSlot{};
};

/// Rebuild neighbor links for the final set of triangles and check edge use is manifold.
/// @param mesh Mesh whose triangle neighbor arrays will be filled.
/// @return True when every vertex index is valid and no edge is used by more than two triangles.
/// @note A boundary edge has one triangle and keeps an empty neighbor slot.
bool BuildFinalAdjacency(ConstrainedMesh& mesh) {
    std::map<Edge, FirstEdgeUse> firstTriangleByEdge;

    // Clear old links and process every edge of every final triangle.
    for (std::size_t triangleIndex = 0; triangleIndex < mesh.triangles.size(); ++triangleIndex) {
        MeshTriangle& triangle = mesh.triangles[triangleIndex];
        triangle.neighbors = {};

        for (std::size_t edgeSlot = 0; edgeSlot < 3; ++edgeSlot) {
            // Neighbor slot 0 is across vertices 1--2, slot 1 across 2--0, and slot 2 across 0--1.
            const std::size_t firstVertex = triangle.vertices[(edgeSlot + 1) % 3];
            const std::size_t secondVertex = triangle.vertices[(edgeSlot + 2) % 3];
            if (firstVertex >= mesh.vertices.size() || secondVertex >= mesh.vertices.size() ||
                firstVertex == secondVertex) return false;

            // Sorting the endpoints makes the same edge match in either direction.
            const Edge edge = MakeEdge(firstVertex, secondVertex);
            const auto [edgeEntry, isFirstUse] = firstTriangleByEdge.emplace(
                edge, FirstEdgeUse{triangleIndex, edgeSlot});

            // One use means a boundary edge. A second use links both triangles.
            if (isFirstUse) continue;

            const FirstEdgeUse firstUse = edgeEntry->second;
            auto& firstNeighbor = mesh.triangles[firstUse.triangleIndex].neighbors[firstUse.edgeSlot];
            if (firstNeighbor) return false; // A third triangle would make this edge non-manifold.

            firstNeighbor = triangleIndex;
            triangle.neighbors[edgeSlot] = firstUse.triangleIndex;
        }
    }
    return true;
}

/// Check whether an undirected edge currently appears in the mesh edge table.
/// @param triangleMesh Mesh to search.
/// @param edge Endpoint indices, in canonical order or either input order.
/// @return True if at least one live triangle uses this edge.
bool HasEdge(const TriangleMesh& triangleMesh, const Edge& edge) { return triangleMesh.trianglesByEdge.contains(edge); }

/// Tell which way three points turn when moving from the first point through the second to the third.
/// @param firstPoint Starting point of the turn.
/// @param secondPoint Point the path reaches next.
/// @param thirdPoint Point the path reaches last.
/// @return Positive for a left turn, negative for a right turn, and zero for a straight line.
double Orient(const Vector2d& firstPoint, const Vector2d& secondPoint, const Vector2d& thirdPoint) {
    return geometry::Orient2D(firstPoint, secondPoint, thirdPoint);
}

/// Check whether a test point lies inside the circle through a triangle's three points.
/// @param firstTrianglePoint First triangle corner.
/// @param secondTrianglePoint Second triangle corner.
/// @param thirdTrianglePoint Third triangle corner; the triangle is expected to be counterclockwise.
/// @param testPoint Point being checked against the circle.
/// @return Positive when the point is inside, negative when outside, and zero when on the circle.
double InBall(const Vector2d& firstTrianglePoint, const Vector2d& secondTrianglePoint,
    const Vector2d& thirdTrianglePoint, const Vector2d& testPoint) {
    return geometry::InCircle2D(firstTrianglePoint, secondTrianglePoint, thirdTrianglePoint, testPoint);
}

/// Return the same triangle with its vertices ordered counterclockwise.
/// @param t Three vertex indices; the input order may go either way.
/// @param points Coordinates used to determine the triangle's direction.
/// @return A copy of `t`, reversing two indices only when it was clockwise.
Triangle Ccw(Triangle t, const std::vector<Vector2d>& points) {
    if (Orient(points[t[0]], points[t[1]], points[t[2]]) < 0.0) std::swap(t[1], t[2]);
    return t;
}

/// Store a contour as indices into the shared point list, rather than copying its coordinates.
struct ContourIndices { std::vector<std::size_t> vertices; };

/// Keep a region's mesh vertices, matching coordinates, and contour index loops together.
/// The same point index refers to the same position in `points` and to its output vertex in `mesh`.
struct InputGeometry {
    ConstrainedMesh mesh;
    std::vector<Vector2d> points;
    std::vector<ContourIndices> contours;
};

/// Check whether a point is strictly inside a contour, using the shared point-in-polygon rule.
/// @param point Position to test.
/// @param contour Contour stored as indices into `points`.
/// @param points Shared coordinate list.
/// @return True only for points inside the contour; a point exactly on its edge counts as outside.
bool PointInLoop(const Vector2d& point, const ContourIndices& contour, const std::vector<Vector2d>& points) {
    std::vector<Vector2d> polygon;
    polygon.reserve(contour.vertices.size());
    for (std::size_t index : contour.vertices) polygon.push_back(points[index]);
    return geometry::PointInPolygon(polygon, point, geometry::BoundaryPolicy::Outside);
}

/// Check whether two line segments cross each other away from their endpoints.
/// @param a First endpoint of the first segment.
/// @param b Second endpoint of the first segment.
/// @param c First endpoint of the second segment.
/// @param d Second endpoint of the second segment.
/// @return True only when each segment's endpoints fall on opposite sides of the other segment.
/// @note Touching at an endpoint or lying along the same line is not a proper crossing here.
bool ProperlyIntersects(const Vector2d& a, const Vector2d& b, const Vector2d& c, const Vector2d& d) {
    const double ab_c = Orient(a, b, c), ab_d = Orient(a, b, d);
    const double cd_a = Orient(c, d, a), cd_b = Orient(c, d, b);
    return ((ab_c > 0.0 && ab_d < 0.0) || (ab_c < 0.0 && ab_d > 0.0)) &&
        ((cd_a > 0.0 && cd_b < 0.0) || (cd_a < 0.0 && cd_b > 0.0));
}

/// Check whether point `p` lies on the closed segment from `a` to `b`.
/// @param a First endpoint of the segment.
/// @param b Second endpoint of the segment.
/// @param p Point being checked.
/// @return True when `p` is on the same line and between both endpoints, including the endpoints.
bool OnSegment(const Vector2d& a, const Vector2d& b, const Vector2d& p) {
    return Orient(a, b, p) == 0.0 && p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x) &&
        p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y);
}

/// Calculate twice the signed area of a polygon whose vertices are point indices.
/// @param polygon Indices in the order they go around the polygon.
/// @param points Coordinates for those indices.
/// @return Positive for counterclockwise order, negative for clockwise order, and zero for no area.
double PolygonArea2(const std::vector<std::size_t>& polygon, const std::vector<Vector2d>& points) {
    std::vector<Vector2d> positions;
    positions.reserve(polygon.size());
    for (std::size_t vertex : polygon) positions.push_back(points[vertex]);
    return geometry::SignedTwiceArea(positions);
}

/// Check whether a point is inside or on the edge of a triangle.
/// @param p Point being checked.
/// @param a First triangle corner.
/// @param b Second triangle corner.
/// @param c Third triangle corner.
/// @return True when the point lies on the same side of all three directed edges, including their lines.
bool PointInTriangleInclusive(const Vector2d& p, const Vector2d& a, const Vector2d& b, const Vector2d& c) {
    const double ab = Orient(a, b, p), bc = Orient(b, c, p), ca = Orient(c, a, p);
    return (ab >= 0.0 && bc >= 0.0 && ca >= 0.0) || (ab <= 0.0 && bc <= 0.0 && ca <= 0.0);
}

/// Split a simple polygon into triangles by repeatedly cutting off a safe corner.
/// @param polygon Polygon vertex indices in boundary order; this local copy may be reversed.
/// @param points Coordinates for all vertex indices.
/// @param result Receives each created triangle; existing entries are preserved.
/// @return True when the whole polygon was split into non-flat triangles; false if no safe cut exists.
/// @note Straight-line boundary vertices are kept in the polygon and are not cut off as corners.
bool TriangulateSimplePolygon(std::vector<std::size_t> polygon, const std::vector<Vector2d>& points,
    std::vector<Triangle>& result) {
    // A polygon needs at least three corners and must enclose some area before it can be split.
    if (polygon.size() < 3) return false;
    const double area2 = PolygonArea2(polygon, points);
    if (area2 == 0.0) return false;
    // Use counterclockwise order so the same positive-turn check works at every candidate corner.
    if (area2 < 0.0) std::reverse(polygon.begin(), polygon.end());
    while (polygon.size() > 3) {
        bool clipped = false;
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            const auto prev = polygon[(i + polygon.size() - 1) % polygon.size()];
            const auto curr = polygon[i];
            const auto next = polygon[(i + 1) % polygon.size()];
            // These three points form a possible cut: curr would be the corner removed from the outline.
            if (Orient(points[prev], points[curr], points[next]) <= 0.0) continue;
            bool containsOther = false;
            for (std::size_t j = 0; j < polygon.size(); ++j) {
                const auto candidate = polygon[j];
                if (candidate == prev || candidate == curr || candidate == next) continue;
                if (PointInTriangleInclusive(points[candidate], points[prev], points[curr], points[next])) {
                    containsOther = true; break;
                }
            }
            // Do not cut this corner if any other remaining vertex would be inside the new triangle.
            if (containsOther) continue;
            result.push_back({prev, curr, next});
            polygon.erase(polygon.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true; break;
        }
        if (!clipped) {
            // No corner could be cut without crossing another remaining vertex.
            return false;
        }
    }
    // The last three vertices are the final face; reject it if they lie on one line.
    if (Orient(points[polygon[0]], points[polygon[1]], points[polygon[2]]) == 0.0) return false;
    result.push_back(Ccw({polygon[0], polygon[1], polygon[2]}, points));
    return true;
}

/// Recover a required edge by removing the strip of triangles it crosses and filling both sides again.
/// @param constraint Endpoint indices of the edge that must appear in the mesh.
/// @param triangleMesh Mesh to update if the replacement is valid.
/// @param points Coordinates for all mesh vertex indices.
/// @param fixedConstraintEdges Edges already recovered and therefore protected from removal.
/// @return True when a valid replacement mesh contains this edge and all protected edges.
/// @note The new mesh is built as a copy first, so a failed attempt leaves the original mesh intact.
bool RecoverByCavity(const Edge& constraint, TriangleMesh& triangleMesh, const std::vector<Vector2d>& points,
    const std::set<Edge>& fixedConstraintEdges, ConstraintRecoveryStats* diagnostics) {
    const auto started = diagnostics ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    if (diagnostics) ++diagnostics->cavityAttempts;
    const auto finish = [&](bool recovered, const char* failure) {
        if (diagnostics) {
            diagnostics->cavityMs += ElapsedMilliseconds(started);
            if (!recovered) diagnostics->cavityFailure = failure;
        }
        return recovered;
    };

    // Find every current mesh edge that the requested straight segment passes through.
    std::vector<Edge> crossedEdges;
    for (const auto& [edge, uses] : triangleMesh.trianglesByEdge) {
        (void)uses;
        if (diagnostics) ++diagnostics->cavityEdgeEntriesScanned;
        if (ProperlyIntersects(points[constraint[0]], points[constraint[1]], points[edge[0]], points[edge[1]])) {
            if (fixedConstraintEdges.contains(edge)) {
                if (diagnostics) ++diagnostics->cavityCrossedEdges;
                return finish(false, "crosses-fixed-constraint");
            }
            crossedEdges.push_back(edge);
        }
    }
    if (diagnostics) diagnostics->cavityCrossedEdges += crossedEdges.size();
    if (crossedEdges.empty()) return finish(false, "no-crossed-edges");

    // Each crossed edge belongs to two triangles. Removing those triangles makes a strip around the segment.
    std::set<TriangleIndex> removed;
    for (const Edge& edge : crossedEdges) {
        const auto it = triangleMesh.trianglesByEdge.find(edge);
        if (it == triangleMesh.trianglesByEdge.end() || it->second.triangleCount != 2)
            return finish(false, "crossed-edge-is-not-interior");
        for (std::size_t i = 0; i < 2; ++i) removed.insert(it->second.triangles[i].triangleIndex);
    }
    if (diagnostics) diagnostics->cavityRemovedTriangles += removed.size();

    // Count edges around the removed triangles: once means on the strip's outside edge;
    // twice means shared inside the strip and therefore not part of its boundary.
    std::map<Edge, int> counts;
    for (TriangleIndex index : removed) {
        const Triangle& t = triangleMesh.triangles[index].vertexIndices;
        for (std::size_t i = 0; i < 3; ++i) ++counts[MakeEdge(t[i], t[(i + 1) % 3])];
    }
    // Turn the outside edges into a small neighbor list so the two boundary routes can be walked.
    std::map<std::size_t, std::vector<std::size_t>> graph;
    for (const auto& [edge, count] : counts) if (count == 1) {
        graph[edge[0]].push_back(edge[1]); graph[edge[1]].push_back(edge[0]);
    }
    if (graph[constraint[0]].size() != 2 || graph[constraint[1]].size() != 2)
        return finish(false, "strip-boundary-endpoint-degree");

    /// Walk one side of the strip boundary, starting from one neighbor of the first constraint endpoint.
    /// @param firstNeighbor First boundary vertex after `constraint[0]`.
    /// @return The ordered path ending at `constraint[1]`, or an empty list if the walk is invalid.
    auto makePath = [&](std::size_t firstNeighbor) -> std::vector<std::size_t> {
        std::vector<std::size_t> path{constraint[0]};
        std::size_t previous = constraint[0], current = firstNeighbor;
        while (true) {
            if (std::find(path.begin(), path.end(), current) != path.end()) {
                return {};
            }
            path.push_back(current);
            if (current == constraint[1]) return path;
            const auto it = graph.find(current);
            if (it == graph.end() || it->second.size() != 2) return {};
            if (path.size() > graph.size()) return {};
            const std::size_t next = it->second[0] == previous ? it->second[1] : it->second[0];
            previous = current; current = next;
        }
    };
    auto first = makePath(graph[constraint[0]][0]);
    if (first.empty()) return finish(false, "first-strip-boundary-path");
    auto second = makePath(graph[constraint[0]][1]);
    if (diagnostics) diagnostics->cavityBoundaryVertices += first.size() + second.size();
    if (second.empty() || first == second) return finish(false, "second-strip-boundary-path");

    // The requested segment divides the removed strip into two polygons; triangulate each side separately.
    std::vector<Triangle> fill;
    const auto pathTriangulationStarted = diagnostics ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    if (!TriangulateSimplePolygon(std::move(first), points, fill)) {
        if (diagnostics) diagnostics->cavityPathMs += ElapsedMilliseconds(pathTriangulationStarted);
        return finish(false, "first-strip-polygon-triangulation");
    }
    if (!TriangulateSimplePolygon(std::move(second), points, fill)) {
        if (diagnostics) diagnostics->cavityPathMs += ElapsedMilliseconds(pathTriangulationStarted);
        return finish(false, "second-strip-polygon-triangulation");
    }
    if (diagnostics) {
        diagnostics->cavityPathMs += ElapsedMilliseconds(pathTriangulationStarted);
        diagnostics->cavityFillTriangles += fill.size();
    }
    // Work on a copy so any failed topology check below cannot damage the current mesh.
    TriangleMesh candidate = triangleMesh;
    for (TriangleIndex index : removed) RemoveTriangle(candidate, index);
    if (!candidate.hasValidTopology) return finish(false, "topology-invalid-after-removal");
    for (const Triangle& t : fill) {
        if (Orient(points[t[0]], points[t[1]], points[t[2]]) == 0.0)
            return finish(false, "degenerate-fill-triangle");
        if (!AddTriangle(candidate, t)) return finish(false, "fill-triangle-topology");
    }
    // Publish the copy only after confirming the requested and previously fixed edges all survived.
    if (!candidate.hasValidTopology || !HasEdge(candidate, constraint))
        return finish(false, "recovered-edge-missing-after-fill");
    for (const Edge& fixed : fixedConstraintEdges)
        if (!HasEdge(candidate, fixed)) return finish(false, "fixed-constraint-missing-after-fill");
    triangleMesh = std::move(candidate);
    return finish(true, "");
}

bool FlipEdge(TriangleMesh& triangleMesh, const Edge& edge, const std::vector<Vector2d>& points);

/// Describe the valid replacement triangles for flipping one shared edge without mutating the mesh.
/// @return True when the edge forms a convex quadrilateral and its replacement edge is not already present.
bool GetFlippedTriangles(const TriangleMesh& triangleMesh, const Edge& edge,
    const std::vector<Vector2d>& points, Triangle& first, Triangle& second, Edge& replacementEdge) {
    const auto edgeIterator = triangleMesh.trianglesByEdge.find(edge);
    if (edgeIterator == triangleMesh.trianglesByEdge.end() || edgeIterator->second.triangleCount != 2) return false;
    const auto adjacentTriangles = edgeIterator->second.triangles;
    const TriangleIndex firstTriangleIndex = adjacentTriangles[0].triangleIndex;
    const TriangleIndex secondTriangleIndex = adjacentTriangles[1].triangleIndex;
    const std::size_t c = triangleMesh.triangles[firstTriangleIndex].vertexIndices[adjacentTriangles[0].oppositeVertexSlot];
    const std::size_t d = triangleMesh.triangles[secondTriangleIndex].vertexIndices[adjacentTriangles[1].oppositeVertexSlot];
    const std::size_t a = edge[0], b = edge[1];
    if (c == d || !ProperlyIntersects(points[a], points[b], points[c], points[d])) return false;

    replacementEdge = MakeEdge(c, d);
    if (triangleMesh.trianglesByEdge.contains(replacementEdge)) return false;
    first = Ccw({c, d, a}, points);
    second = Ccw({d, c, b}, points);
    return Orient(points[first[0]], points[first[1]], points[first[2]]) != 0.0 &&
        Orient(points[second[0]], points[second[1]], points[second[2]]) != 0.0;
}

/// Try to recover one constraint segment by flipping crossing edges, then use cavity recovery if needed.
/// @param constraint Endpoint indices of the single segment to insert.
/// @param triangleMesh Mesh that may be changed by successful flips or cavity filling.
/// @param points Coordinates for all vertex indices.
/// @param fixedConstraintEdges Edges that must not be flipped or removed.
/// @return True when the segment exists and has been added to the protected-edge set.
/// @note The bounded flip loop prevents recovery from running forever; cavity recovery is the fallback.
bool RecoverConstraintSingle(const Edge& constraint, TriangleMesh& triangleMesh,
    const std::vector<Vector2d>& points, std::set<Edge>& fixedConstraintEdges,
    ConstraintRecoveryStats* diagnostics) {
    const auto started = diagnostics ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    const std::size_t iterationLimit = triangleMesh.trianglesByEdge.size() * triangleMesh.trianglesByEdge.size() + 32;
    constexpr std::size_t maximumNonProgressFlipStreak = 8;
    std::size_t nonProgressFlipStreak = 0;
    bool earlyCavityAttempted = false;
    if (diagnostics) diagnostics->iterationLimit = iterationLimit;
    const auto finish = [&](bool recovered, const char* outcome) {
        if (diagnostics) {
            diagnostics->outcome = outcome;
            diagnostics->elapsedMs = ElapsedMilliseconds(started);
        }
        return recovered;
    };

    for (std::size_t iteration = 0; iteration < iterationLimit; ++iteration) {
        if (diagnostics) ++diagnostics->iterations;
        if (HasEdge(triangleMesh, constraint)) {
            fixedConstraintEdges.insert(constraint);
            return finish(true, diagnostics && diagnostics->successfulFlips > 0 ? "recovered-by-flips" : "already-present");
        }
        std::vector<Edge> crossingEdges;
        for (const auto& [edge, trianglesOnEdge] : triangleMesh.trianglesByEdge) {
            (void)trianglesOnEdge;
            if (diagnostics) {
                ++diagnostics->edgeEntriesScanned;
                if (edge != constraint) ++diagnostics->intersectionTests;
            }
            if (edge != constraint && ProperlyIntersects(points[constraint[0]], points[constraint[1]], points[edge[0]], points[edge[1]])) crossingEdges.push_back(edge);
        }
        if (diagnostics) diagnostics->crossingEdgesSeen += crossingEdges.size();
        bool flipped = false;
        bool madeProgress = false;
        std::vector<Edge> progressCandidates;
        std::vector<Edge> nonProgressCandidates;
        for (const Edge& edge : crossingEdges) if (!fixedConstraintEdges.contains(edge)) {
            Triangle first{}, second{};
            Edge replacementEdge{};
            if (!GetFlippedTriangles(triangleMesh, edge, points, first, second, replacementEdge)) continue;
            const bool replacementStillCrosses = ProperlyIntersects(
                points[constraint[0]], points[constraint[1]],
                points[replacementEdge[0]], points[replacementEdge[1]]);
            (replacementStillCrosses ? nonProgressCandidates : progressCandidates).push_back(edge);
        }
        // Prefer a flip that removes a crossing. If none exists, try neutral flips so
        // they can expose a progressing flip, but bound how long that sequence can run.
        const auto tryFlips = [&](const std::vector<Edge>& candidates, bool candidateMakesProgress) {
            for (const Edge& edge : candidates) {
                if (diagnostics) ++diagnostics->flipAttempts;
                if (!FlipEdge(triangleMesh, edge, points)) continue;
                if (diagnostics) ++diagnostics->successfulFlips;
                madeProgress = candidateMakesProgress;
                flipped = true;
                return;
            }
        };
        tryFlips(progressCandidates, true);
        if (!flipped) tryFlips(nonProgressCandidates, false);
        if (!flipped) {
            if (RecoverByCavity(constraint, triangleMesh, points, fixedConstraintEdges, diagnostics)) {
                fixedConstraintEdges.insert(constraint);
                return finish(true, "recovered-by-cavity");
            }
            return finish(false, "cavity-failed-no-flippable-edge");
        }
        if (madeProgress) {
            nonProgressFlipStreak = 0;
            if (diagnostics) ++diagnostics->progressFlips;
        } else {
            ++nonProgressFlipStreak;
            if (diagnostics) {
                ++diagnostics->nonProgressFlips;
                diagnostics->maximumNonProgressFlipStreak =
                    std::max(diagnostics->maximumNonProgressFlipStreak, nonProgressFlipStreak);
            }
        }
        if (!earlyCavityAttempted && nonProgressFlipStreak >= maximumNonProgressFlipStreak) {
            earlyCavityAttempted = true;
            if (diagnostics) diagnostics->earlyCavityAttempted = true;
            if (RecoverByCavity(constraint, triangleMesh, points, fixedConstraintEdges, diagnostics)) {
                fixedConstraintEdges.insert(constraint);
                if (diagnostics) diagnostics->earlyCavityRecovered = true;
                return finish(true, "recovered-by-early-cavity");
            }
        }
    }
    if (RecoverByCavity(constraint, triangleMesh, points, fixedConstraintEdges, diagnostics)) {
        fixedConstraintEdges.insert(constraint);
        return finish(true, "recovered-by-cavity-after-limit");
    }
    return finish(false, "cavity-failed-after-iteration-limit");
}

/// Replace the shared diagonal of two adjacent triangles with the other diagonal of their four-corner shape.
/// @param triangleMesh Mesh containing the two triangles.
/// @param edge Shared edge to replace.
/// @param points Coordinates for all vertex indices.
/// @return True if the replacement was valid and added; false if the edge cannot be flipped safely.
bool FlipEdge(TriangleMesh& triangleMesh, const Edge& edge, const std::vector<Vector2d>& points) {
    Triangle first{}, second{};
    Edge replacementEdge{};
    if (!GetFlippedTriangles(triangleMesh, edge, points, first, second, replacementEdge)) return false;
    // Remove the old pair, then register the replacement pair and all local links.
    const auto edgeIterator = triangleMesh.trianglesByEdge.find(edge);
    const auto adjacentTriangles = edgeIterator->second.triangles;
    const TriangleIndex firstTriangleIndex = adjacentTriangles[0].triangleIndex;
    const TriangleIndex secondTriangleIndex = adjacentTriangles[1].triangleIndex;
    RemoveTriangle(triangleMesh, firstTriangleIndex); RemoveTriangle(triangleMesh, secondTriangleIndex);
    if (!AddTriangle(triangleMesh, first) || !AddTriangle(triangleMesh, second)) { triangleMesh.hasValidTopology = false; return false; }
    return true;
}

/// Find the outside outline of all points, going counterclockwise around it.
/// @param points All input coordinates. Interior points are allowed and remain in this vector.
/// @param hull Receives input point indices around the outside boundary.
/// @return True when a non-flat hull can be made; false for too few points, duplicate coordinates, or a line-only set.
/// @note Points inside the outline are not returned. Points lying along a straight outside edge are kept.
bool BuildConvexHullBoundary(const std::vector<Vector2d>& points, std::vector<std::size_t>& hull) {
    // Start with an empty result so a failed call cannot leave old hull points behind.
    hull.clear();
    if (points.size() < 3) return false;

    // Sort point numbers, not the points themselves. This puts every point in a known
    // left-to-right order so the two scans below can trace the bottom and top outlines.
    std::vector<std::size_t> sorted(points.size());
    for (std::size_t i = 0; i < sorted.size(); ++i) sorted[i] = i;
    /// Compare two point indices by x, then y, then original index for a stable left-to-right order.
    std::sort(sorted.begin(), sorted.end(), [&](std::size_t a, std::size_t b) {
        // Compare x first; when x matches, compare y. The point number breaks any final tie.
        if (points[a].x != points[b].x) return points[a].x < points[b].x;
        if (points[a].y != points[b].y) return points[a].y < points[b].y;
        return a < b;
    });
    for (std::size_t i = 1; i < sorted.size(); ++i) {
        const Vector2d& previous = points[sorted[i - 1]];
        const Vector2d& current = points[sorted[i]];
        if (previous.x == current.x && previous.y == current.y) {
            return false;
        }
    }

    // The left-to-right scan builds the bottom edge. The right-to-left scan builds the top.
    // Each list holds the point numbers that currently make up that part of the outline.
    std::vector<std::size_t> lower, upper;
    lower.reserve(sorted.size());
    upper.reserve(sorted.size());
    for (std::size_t index : sorted) {
        while (lower.size() >= 2 &&
            // Check the last two kept points and this new point. A right turn or a straight
            // line means the middle point is not a bottom-outline corner, so remove it.
            Orient(points[lower[lower.size() - 2]], points[lower.back()], points[index]) <= 0.0)
            lower.pop_back();
        lower.push_back(index);
    }
    for (auto it = sorted.rbegin(); it != sorted.rend(); ++it) {
        const std::size_t index = *it;
        while (upper.size() >= 2 &&
            // Do the same check while scanning from right to left to build the top outline.
            Orient(points[upper[upper.size() - 2]], points[upper.back()], points[index]) <= 0.0)
            upper.pop_back();
        upper.push_back(index);
    }
    // Each scan ends at a point already present at the start of the other scan.
    // Remove those duplicate ends, then join the two lists into one trip around the outline.
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    if (lower.size() < 3) return false;

    // The scans above keep only the corners. Now add back input points that lie between
    // two corners on a straight outside edge, so those boundary vertices are not lost.
    hull.reserve(points.size());
    for (std::size_t edgeIndex = 0; edgeIndex < lower.size(); ++edgeIndex) {
        const std::size_t first = lower[edgeIndex];
        const std::size_t second = lower[(edgeIndex + 1) % lower.size()];
        hull.push_back(first);

        // Find all input points between these two edge ends. They are found in input order,
        // which may go back and forth along the edge, so collect them before ordering them.
        std::vector<std::size_t> edgePoints;
        for (std::size_t candidate = 0; candidate < points.size(); ++candidate) {
            if (candidate != first && candidate != second &&
                OnSegment(points[first], points[second], points[candidate]))
                edgePoints.push_back(candidate);
        }
        // Pick the coordinate that changes more along the edge. Ordering by that coordinate
        // gives a clear along-edge order (including for nearly vertical or horizontal edges).
        const Vector2d direction = points[second] - points[first];
        const bool sortByX = std::abs(direction.x) >= std::abs(direction.y);
        // Match the edge's travel direction: sort small-to-large if this coordinate increases,
        // and large-to-small if it decreases from first to second.
        const bool ascending = sortByX ? direction.x > 0.0 : direction.y > 0.0;
        /// Put two points on this edge in the order they appear from `first` to `second`.
        std::sort(edgePoints.begin(), edgePoints.end(), [&](std::size_t a, std::size_t b) {
            // Compare the chosen coordinate so the points appear in first-to-second order.
            const double coordinateA = sortByX ? points[a].x : points[a].y;
            const double coordinateB = sortByX ? points[b].x : points[b].y;
            return ascending ? coordinateA < coordinateB : coordinateA > coordinateB;
        });
        // Append them after this edge's first corner. The next loop pass appends the next corner,
        // keeping the complete hull boundary in order without changing its direction.
        hull.insert(hull.end(), edgePoints.begin(), edgePoints.end());
    }
    return true;
}

/// Check shared, non-constraint edges and flip any that violate the Delaunay circle rule.
/// @param triangleMesh Mesh whose edges may be flipped.
/// @param points Coordinates for all vertex indices.
/// @param fixedConstraintEdges Edges that must remain as input contour boundaries.
/// @param diagnostics Optional counters and timing records; null when profiling is disabled.
/// @param stage Label identifying the legalization pass in diagnostic output.
/// @return True when the work list empties with valid topology; false if the safety limit is reached.
bool Legalize(TriangleMesh& triangleMesh, const std::vector<Vector2d>& points,
    const std::set<Edge>& fixedConstraintEdges, CdtDiagnostics* diagnostics, const char* stage);

/// Build a Delaunay triangulation from the real hull, then insert every point inside it.
/// @param input All input coordinates, including hull and interior points.
/// @param points Receives the same coordinate list so returned triangle indices refer to it.
/// @return Triangles using only input points, or an empty list if a required stage fails.
/// @note The real convex hull stays as the mesh boundary; no temporary enclosing triangle is used.
std::vector<Triangle> BowyerWatson(const std::vector<Vector2d>& input, std::vector<Vector2d>& points,
    CdtDiagnostics* diagnostics) {
    const auto started = diagnostics && diagnostics->enabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    const auto fail = [&](const char* reason) -> std::vector<Triangle> {
        if (diagnostics && diagnostics->enabled) {
            diagnostics->bowyerFailure = reason;
            diagnostics->bowyerWatsonMs = ElapsedMilliseconds(started);
        }
        return {};
    };
    points = input;
    if (points.size() < 3) return fail("fewer-than-three-points");
    double minimumX = points[0].x, maximumX = minimumX, minimumY = points[0].y, maximumY = minimumY;
    for (const Vector2d& p : points) {
        minimumX = std::min(minimumX, p.x); maximumX = std::max(maximumX, p.x);
        minimumY = std::min(minimumY, p.y); maximumY = std::max(maximumY, p.y);
    }
    const double boundingBoxSpan = std::max(maximumX - minimumX, maximumY - minimumY);
    // Reject a collapsed or non-finite point cloud before geometric predicates use it.
    if (!std::isfinite(boundingBoxSpan) || boundingBoxSpan <= 0.0) return fail("collapsed-or-nonfinite-bounds");
    const std::size_t n = points.size();
    TriangleMesh triangleMesh;

    std::vector<std::size_t> hull;
    auto stageStarted = diagnostics && diagnostics->enabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    if (!BuildConvexHullBoundary(points, hull)) return fail("convex-hull-build");
    if (diagnostics && diagnostics->enabled) {
        diagnostics->hullBuildMs = ElapsedMilliseconds(stageStarted);
        diagnostics->hullVertices = hull.size();
    }

    std::vector<Triangle> hullTriangles;
    stageStarted = diagnostics && diagnostics->enabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    if (!TriangulateSimplePolygon(hull, points, hullTriangles)) return fail("convex-hull-triangulation");
    if (diagnostics && diagnostics->enabled)
        diagnostics->hullTriangulationMs = ElapsedMilliseconds(stageStarted);
    for (const Triangle& triangle : hullTriangles)
        if (!AddTriangle(triangleMesh, triangle)) return fail("convex-hull-topology");
    const std::set<Edge> noFixedConstraints;
    if (!Legalize(triangleMesh, points, noFixedConstraints, diagnostics, "bowyer-hull-seed"))
        return fail("convex-hull-seed-legalization");
    // Mark hull indices so they are not inserted a second time below.
    std::vector<bool> isHullVertex(n, false);
    for (std::size_t vertex : hull) isHullVertex[vertex] = true;

    // Point insertion: remove triangles whose circumcircles contain the point, then fill the cavity
    // from edges that occur only once on the removed triangles' boundary.
    for (std::size_t pointIndex = 0; pointIndex < n; ++pointIndex) {
        if (isHullVertex[pointIndex]) continue;
        const auto insertionStarted = diagnostics && diagnostics->enabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
        const std::size_t storedSlotsAtScan = triangleMesh.triangles.size();
        if (diagnostics && diagnostics->enabled) {
            ++diagnostics->interiorPointCandidates;
            diagnostics->maximumStoredSlotsAtScan = std::max(diagnostics->maximumStoredSlotsAtScan, storedSlotsAtScan);
            diagnostics->storedTriangleSlotsScanned += storedSlotsAtScan;
        }
        std::vector<bool> trianglesToRemove(triangleMesh.triangles.size(), false);
        std::map<Edge, int> boundaryEdgeCounts;
        bool foundCavityTriangle = false;
        std::size_t liveTrianglesScanned = 0;
        std::size_t cavityTriangleCount = 0;
        std::size_t boundaryEdgeCount = 0;
        std::size_t trianglesAdded = 0;
        // A triangle is removed when this new point lies inside its circumcircle.
        for (std::size_t triangleIndex = 0; triangleIndex < triangleMesh.triangles.size(); ++triangleIndex) {
            if (!triangleMesh.triangles[triangleIndex].isPresent) continue;
            if (diagnostics && diagnostics->enabled) ++liveTrianglesScanned;
            const Triangle& t = triangleMesh.triangles[triangleIndex].vertexIndices;
            if (diagnostics && diagnostics->enabled) ++diagnostics->circumcircleTests;
            if (InBall(points[t[0]], points[t[1]], points[t[2]], points[pointIndex]) > 0.0) {
                trianglesToRemove[triangleIndex] = true; foundCavityTriangle = true;
                if (diagnostics && diagnostics->enabled) ++cavityTriangleCount;
                // An edge counted twice is internal to the removed cavity; an edge counted once is on its border.
                for (int edgeIndex = 0; edgeIndex < 3; ++edgeIndex) ++boundaryEdgeCounts[MakeEdge(t[edgeIndex], t[(edgeIndex + 1) % 3])];
            }
        }
        if (!foundCavityTriangle) {
            if (diagnostics && diagnostics->enabled) {
                ++diagnostics->pointsWithoutCavity;
                diagnostics->liveTrianglesScanned += liveTrianglesScanned;
                diagnostics->slowInsertions.push_back({pointIndex, points[pointIndex], storedSlotsAtScan,
                    liveTrianglesScanned, 0, 0, 0, ElapsedMilliseconds(insertionStarted)});
            }
            continue;
        }
        if (diagnostics && diagnostics->enabled) diagnostics->cavityTrianglesRemoved += cavityTriangleCount;
        // Remove the old cavity faces before filling their exposed boundary with faces to the new point.
        for (std::size_t triangleIndex = 0; triangleIndex < trianglesToRemove.size(); ++triangleIndex)
            if (trianglesToRemove[triangleIndex]) RemoveTriangle(triangleMesh, triangleIndex);
        for (const auto& [edge, count] : boundaryEdgeCounts) if (count == 1) {
            if (diagnostics && diagnostics->enabled) ++boundaryEdgeCount;
            Triangle t = Ccw({edge[0], edge[1], pointIndex}, points);
            if (Orient(points[t[0]], points[t[1]], points[t[2]]) != 0.0 && AddTriangle(triangleMesh, t))
                if (diagnostics && diagnostics->enabled) ++trianglesAdded;
        }
        if (diagnostics && diagnostics->enabled) {
            diagnostics->liveTrianglesScanned += liveTrianglesScanned;
            diagnostics->cavityBoundaryEdges += boundaryEdgeCount;
            diagnostics->pointTrianglesAdded += trianglesAdded;
            diagnostics->slowInsertions.push_back({pointIndex, points[pointIndex], storedSlotsAtScan,
                liveTrianglesScanned, cavityTriangleCount, boundaryEdgeCount, trianglesAdded,
                ElapsedMilliseconds(insertionStarted)});
        }
        // A failed face addition marks the topology invalid; stop rather than insert more points into it.
        if (!triangleMesh.hasValidTopology) break;
    }
    std::vector<Triangle> triangles;
    for (const StoredTriangle& triangle : triangleMesh.triangles) {
        if (!triangle.isPresent) continue;
        triangles.push_back(triangle.vertexIndices);
    }
    if (diagnostics && diagnostics->enabled) {
        diagnostics->finalTriangleSlots = triangleMesh.triangles.size();
        diagnostics->finalLiveTriangles = triangles.size();
    }
    if (!triangleMesh.hasValidTopology) return fail("invalid-topology-after-point-insertion");
    if (diagnostics && diagnostics->enabled) diagnostics->bowyerWatsonMs = ElapsedMilliseconds(started);
    return triangles;
}

/// Recover a full contour edge, splitting it first wherever another input point lies on it.
/// @param constraint Original contour edge endpoints.
/// @param triangleMesh Mesh to update while recovering its pieces.
/// @param points Coordinates used to find points lying along the edge.
/// @param fixedConstraintEdges Recovered edges that must remain unchanged.
/// @return True when every consecutive piece of the contour edge was recovered.
/// @note Splitting avoids asking the mesh to contain a long edge that skips over an existing vertex.
bool RecoverConstraint(const Edge& constraint, TriangleMesh& triangleMesh,
    const std::vector<Vector2d>& points, std::set<Edge>& fixedConstraintEdges,
    CdtDiagnostics* diagnostics, std::size_t constraintIndex) {
    std::vector<std::size_t> splitPoints{constraint[0], constraint[1]};
    std::size_t pointChecks = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (i == constraint[0] || i == constraint[1]) continue;
        if (diagnostics && diagnostics->enabled) ++pointChecks;
        if (OnSegment(points[constraint[0]], points[constraint[1]], points[i]))
            splitPoints.push_back(i);
    }
    // Order the endpoints and any points on the segment from the constraint's first end to its second.
    const Vector2d direction = points[constraint[1]] - points[constraint[0]];
    const bool sortX = std::abs(direction.x) >= std::abs(direction.y);
    /// Compare two points along the constraint using the coordinate that changes most.
    std::sort(splitPoints.begin(), splitPoints.end(), [&](std::size_t a, std::size_t b) {
        return sortX ? (direction.x >= 0.0 ? points[a].x < points[b].x : points[a].x > points[b].x)
                     : (direction.y >= 0.0 ? points[a].y < points[b].y : points[a].y > points[b].y);
    });
    splitPoints.erase(std::unique(splitPoints.begin(), splitPoints.end()), splitPoints.end());
    const std::size_t segmentCount = splitPoints.size() - 1;
    if (diagnostics && diagnostics->enabled) {
        ++diagnostics->originalConstraintCount;
        diagnostics->constraintPointChecks += pointChecks;
        diagnostics->constraintSegments += segmentCount;
    }
    // Recover one short segment at a time so every point on this contour edge becomes a mesh endpoint.
    for (std::size_t i = 1; i < splitPoints.size(); ++i) {
        const Edge segment = MakeEdge(splitPoints[i - 1], splitPoints[i]);
        ConstraintRecoveryStats segmentDiagnostics;
        ConstraintRecoveryStats* segmentDiagnosticsPtr = nullptr;
        if (diagnostics && diagnostics->enabled) {
            segmentDiagnostics.originalConstraint = constraint;
            segmentDiagnostics.segment = segment;
            segmentDiagnostics.segmentStart = points[segment[0]];
            segmentDiagnostics.segmentEnd = points[segment[1]];
            segmentDiagnostics.constraintIndex = constraintIndex;
            segmentDiagnostics.segmentIndex = i - 1;
            segmentDiagnostics.segmentCount = segmentCount;
            segmentDiagnostics.pointChecks = pointChecks;
            segmentDiagnosticsPtr = &segmentDiagnostics;
        }
        const bool recovered = RecoverConstraintSingle(segment, triangleMesh, points, fixedConstraintEdges,
            segmentDiagnosticsPtr);
        if (diagnostics && diagnostics->enabled) {
            diagnostics->constraintRecoveryIterationCount += segmentDiagnostics.iterations;
            diagnostics->constraintEdgeEntriesScanned += segmentDiagnostics.edgeEntriesScanned;
            diagnostics->constraintIntersectionTests += segmentDiagnostics.intersectionTests;
            diagnostics->constraintCrossingEdgesSeen += segmentDiagnostics.crossingEdgesSeen;
            diagnostics->constraintFlipAttempts += segmentDiagnostics.flipAttempts;
            diagnostics->constraintSuccessfulFlips += segmentDiagnostics.successfulFlips;
            diagnostics->constraintProgressFlips += segmentDiagnostics.progressFlips;
            diagnostics->constraintNonProgressFlips += segmentDiagnostics.nonProgressFlips;
            diagnostics->earlyCavityAttempts += segmentDiagnostics.earlyCavityAttempted ? 1 : 0;
            diagnostics->earlyCavitySuccesses += segmentDiagnostics.earlyCavityRecovered ? 1 : 0;
            diagnostics->cavityFallbackAttempts += segmentDiagnostics.cavityAttempts;
            if (recovered) ++diagnostics->recoveredSegments;
            diagnostics->recoverySegments.push_back(std::move(segmentDiagnostics));
        }
        if (!recovered) return false;
    }
    return true;
}

/// Improve the triangulation by flipping non-fixed shared edges that fail the circle rule.
/// @param triangleMesh Mesh to legalize.
/// @param points Coordinates for all vertex indices.
/// @param fixedConstraintEdges Contour edges that must not be changed.
/// @return True when every needed check is complete and topology is valid; false if the safety limit is hit.
bool Legalize(TriangleMesh& triangleMesh, const std::vector<Vector2d>& points,
    const std::set<Edge>& fixedConstraintEdges, CdtDiagnostics* diagnostics, const char* stage) {
    const auto started = diagnostics && diagnostics->enabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    const std::size_t iterationLimit = triangleMesh.trianglesByEdge.size() * triangleMesh.trianglesByEdge.size() * 8 + 64;
    // Legalization: check shared, unconstrained edges and flip those that violate the Delaunay circle rule.
    // Changed faces add their edges back to the worklist; the limit prevents an endless loop.
    std::deque<Edge> edgesToCheck;
    // Start with every current edge. Newly changed faces add their edges back for another check.
    for (const auto& [edge, trianglesOnEdge] : triangleMesh.trianglesByEdge) { (void)trianglesOnEdge; edgesToCheck.push_back(edge); }
    const std::size_t initialQueueSize = edgesToCheck.size();
    std::size_t maximumQueueSize = initialQueueSize;
    std::size_t checkedEdgeCount = 0;
    std::size_t successfulFlips = 0;
    while (!edgesToCheck.empty() && checkedEdgeCount++ < iterationLimit) {
        const Edge edge = edgesToCheck.front(); edgesToCheck.pop_front();
        auto it = triangleMesh.trianglesByEdge.find(edge);
        if (it == triangleMesh.trianglesByEdge.end() || it->second.triangleCount != 2 || fixedConstraintEdges.contains(edge)) continue;
        // The opposite corners are the vertices not on the shared edge, one from each adjacent triangle.
        const auto adjacentTriangles = it->second.triangles;
        const std::size_t c = triangleMesh.triangles[adjacentTriangles[0].triangleIndex].vertexIndices[adjacentTriangles[0].oppositeVertexSlot];
        const std::size_t d = triangleMesh.triangles[adjacentTriangles[1].triangleIndex].vertexIndices[adjacentTriangles[1].oppositeVertexSlot];
        if (c == d || !ProperlyIntersects(points[edge[0]], points[edge[1]], points[c], points[d])) continue;
        // If d is inside the circumcircle through this edge and c, replacing the shared edge improves the mesh.
        Triangle t = Ccw({edge[0], edge[1], c}, points);
        if (InBall(points[t[0]], points[t[1]], points[t[2]], points[d]) <= 0.0) continue;
        const std::size_t old_triangle_count = triangleMesh.triangles.size();
        if (!FlipEdge(triangleMesh, edge, points)) continue;
        if (diagnostics && diagnostics->enabled) ++successfulFlips;
        // Recheck the changed faces' three edges; these include every edge whose opposite face changed.
        for (std::size_t triangleIndex = old_triangle_count; triangleIndex < triangleMesh.triangles.size(); ++triangleIndex) {
            const StoredTriangle& triangle = triangleMesh.triangles[triangleIndex];
            for (std::size_t opposite = 0; opposite < 3; ++opposite)
                edgesToCheck.push_back(MakeEdge(triangle.vertexIndices[(opposite + 1) % 3], triangle.vertexIndices[(opposite + 2) % 3]));
        }
        if (diagnostics && diagnostics->enabled)
            maximumQueueSize = std::max(maximumQueueSize, edgesToCheck.size());
    }
    const bool reachedLimit = !edgesToCheck.empty();
    if (diagnostics && diagnostics->enabled) {
        diagnostics->legalizations.push_back({stage, initialQueueSize,
            std::min(checkedEdgeCount, iterationLimit), successfulFlips, maximumQueueSize,
            iterationLimit, reachedLimit, ElapsedMilliseconds(started)});
    }
    if (!reachedLimit) return triangleMesh.hasValidTopology;
    // End: the safety iterationLimit was reached before the mesh settled.
    return false;
}

/// Collect every required edge around every contour in this region.
/// @param input Region geometry whose contours are stored as point indices.
/// @param constraints Set that receives the undirected contour edges; existing entries are kept.
/// @return No value. Each contour's final vertex is connected back to its first vertex.
void BuildConstraints(const InputGeometry& input, std::set<Edge>& constraints) {
    // Each vertex connects to the next one in contour order. The modulo wraps the last vertex to the first.
    for (const ContourIndices& contour : input.contours) for (std::size_t i = 0; i < contour.vertices.size(); ++i) {
        const std::size_t startVertexIndex = contour.vertices[i], endVertexIndex = contour.vertices[(i + 1) % contour.vertices.size()];
        constraints.insert(MakeEdge(startVertexIndex, endVertexIndex));
    }
}

/// Build one region's triangulation, recover its contour edges, and legalize its remaining edges.
/// @param input Region points, output vertices, and contour loops.
/// @param constraints Edges that the final triangulation must contain.
/// @param triangles Receives the live triangles after successful recovery and legalization.
/// @param error Receives a short description of the first failed stage.
/// @return True when all stages succeed; false as soon as one stage fails.
bool BuildConstrainedTriangulation(const InputGeometry& input, const std::set<Edge>& constraints,
    std::vector<Triangle>& triangles, std::string& error, CdtDiagnostics* diagnostics) {
    const auto started = diagnostics && diagnostics->enabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    const auto fail = [&](const char* stage) {
        if (diagnostics && diagnostics->enabled) {
            diagnostics->constrainedBuildFailure = stage;
            diagnostics->constrainedBuildMs = ElapsedMilliseconds(started);
        }
        return false;
    };
    // First make the unconstrained Delaunay mesh, then add contour edges, then improve only free edges.
    std::vector<Vector2d> points;
    std::vector<Triangle> initialTriangles = BowyerWatson(input.points, points, diagnostics);
    if (initialTriangles.empty()) {
        error = "Could not triangulate input vertices.";
        return fail("bowyer-watson");
    }
    TriangleMesh triangleMesh;
    auto stageStarted = diagnostics && diagnostics->enabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    for (const Triangle& triangle : initialTriangles) {
        if (!AddTriangle(triangleMesh, triangle)) {
            error = "Bowyer-Watson faces could not form valid adjacency.";
            if (diagnostics && diagnostics->enabled)
                diagnostics->seedAdjacencyMs = ElapsedMilliseconds(stageStarted);
            return fail("seed-adjacency");
        }
    }
    if (diagnostics && diagnostics->enabled)
        diagnostics->seedAdjacencyMs = ElapsedMilliseconds(stageStarted);

    std::set<Edge> fixedConstraintEdges;
    stageStarted = diagnostics && diagnostics->enabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    std::size_t constraintIndex = 0;
    for (const Edge& edge : constraints) {
        if (!RecoverConstraint(edge, triangleMesh, points, fixedConstraintEdges, diagnostics, constraintIndex)) {
            if (diagnostics && diagnostics->enabled)
                diagnostics->constraintRecoveryMs = ElapsedMilliseconds(stageStarted);
            error = "Could not recover every contour constraint.";
            return fail("constraint-recovery");
        }
        ++constraintIndex;
    }
    if (diagnostics && diagnostics->enabled)
        diagnostics->constraintRecoveryMs = ElapsedMilliseconds(stageStarted);
    if (!Legalize(triangleMesh, points, fixedConstraintEdges, diagnostics, "constrained-final")) {
        error = "Constrained edge legalization did not converge.";
        return fail("constrained-final-legalization");
    }
    triangles.clear();
    // Removed faces remain in storage, so return only entries still marked as present.
    for (const StoredTriangle& triangle : triangleMesh.triangles)
        if (triangle.isPresent) triangles.push_back(triangle.vertexIndices);
    if (diagnostics && diagnostics->enabled)
        diagnostics->constrainedBuildMs = ElapsedMilliseconds(started);
    return true;
}

void PrintCdtRegionDiagnostics(std::size_t regionIndex, const InputGeometry& input,
    std::size_t contourCount, std::size_t constraintCount, const CdtDiagnostics& diagnostics,
    double inputAssemblyMs, double filterMs, double appendMs, double regionMs,
    std::size_t generatedTriangleCount, const char* outcome) {
    if (!diagnostics.enabled) return;

    std::ostringstream output;
    output << std::fixed << std::setprecision(3);
    output << "[navmesh-cdt] region=" << regionIndex << " result=" << outcome
        << " contours=" << contourCount << " input_points=" << input.points.size()
        << " constraints=" << constraintCount << " generated_triangles=" << generatedTriangleCount
        << " input_ms=" << inputAssemblyMs << " build_ms=" << diagnostics.constrainedBuildMs
        << " filter_ms=" << filterMs << " append_ms=" << appendMs << " region_ms=" << regionMs;
    if (!diagnostics.constrainedBuildFailure.empty())
        output << " failure_stage=" << diagnostics.constrainedBuildFailure;
    if (!diagnostics.bowyerFailure.empty())
        output << " bowyer_failure=" << diagnostics.bowyerFailure;
    output << '\n';

    const double scanLiveRatio = diagnostics.liveTrianglesScanned == 0 ? 0.0 :
        static_cast<double>(diagnostics.storedTriangleSlotsScanned) /
            static_cast<double>(diagnostics.liveTrianglesScanned);
    output << "[navmesh-cdt] region=" << regionIndex << " stage=bowyer-watson"
        << " total_ms=" << diagnostics.bowyerWatsonMs << " hull_ms=" << diagnostics.hullBuildMs
        << " hull_vertices=" << diagnostics.hullVertices
        << " hull_triangulation_ms=" << diagnostics.hullTriangulationMs
        << " interior_point_candidates=" << diagnostics.interiorPointCandidates
        << " points_without_cavity=" << diagnostics.pointsWithoutCavity
        << " triangle_slots_scanned=" << diagnostics.storedTriangleSlotsScanned
        << " live_triangles_scanned=" << diagnostics.liveTrianglesScanned
        << " circumcircle_tests=" << diagnostics.circumcircleTests
        << " cavity_triangles_removed=" << diagnostics.cavityTrianglesRemoved
        << " cavity_boundary_edges=" << diagnostics.cavityBoundaryEdges
        << " triangles_added=" << diagnostics.pointTrianglesAdded
        << " max_stored_slots_at_scan=" << diagnostics.maximumStoredSlotsAtScan
        << " final_triangle_slots=" << diagnostics.finalTriangleSlots
        << " final_live_triangles=" << diagnostics.finalLiveTriangles
        << " stored_to_live_scan_ratio=" << scanLiveRatio << '\n';

    output << "[navmesh-cdt] region=" << regionIndex << " stage=constraint-recovery"
        << " total_ms=" << diagnostics.constraintRecoveryMs
        << " original_constraints=" << diagnostics.originalConstraintCount
        << " split_segments=" << diagnostics.constraintSegments
        << " recovered_segments=" << diagnostics.recoveredSegments
        << " point_on_segment_checks=" << diagnostics.constraintPointChecks
        << " iterations=" << diagnostics.constraintRecoveryIterationCount
        << " edge_entries_scanned=" << diagnostics.constraintEdgeEntriesScanned
        << " intersection_tests=" << diagnostics.constraintIntersectionTests
        << " crossing_edges_seen=" << diagnostics.constraintCrossingEdgesSeen
        << " flip_attempts=" << diagnostics.constraintFlipAttempts
        << " successful_flips=" << diagnostics.constraintSuccessfulFlips
        << " progress_flips=" << diagnostics.constraintProgressFlips
        << " non_progress_flips=" << diagnostics.constraintNonProgressFlips
        << " early_cavity_attempts=" << diagnostics.earlyCavityAttempts
        << " early_cavity_successes=" << diagnostics.earlyCavitySuccesses
        << " cavity_fallback_attempts=" << diagnostics.cavityFallbackAttempts << '\n';

    for (const LegalizationStats& legalization : diagnostics.legalizations) {
        output << "[navmesh-cdt] region=" << regionIndex << " stage=legalize"
            << " pass=" << legalization.stage << " elapsed_ms=" << legalization.elapsedMs
            << " initial_queue=" << legalization.initialQueueSize
            << " checked_edges=" << legalization.checkedEdges
            << " successful_flips=" << legalization.successfulFlips
            << " max_queue=" << legalization.maximumQueueSize
            << " iteration_limit=" << legalization.iterationLimit
            << " reached_limit=" << (legalization.reachedLimit ? "true" : "false") << '\n';
    }

    std::vector<BowyerInsertionStats> slowInsertions = diagnostics.slowInsertions;
    std::stable_sort(slowInsertions.begin(), slowInsertions.end(), [](const auto& a, const auto& b) {
        return a.elapsedMs > b.elapsedMs;
    });
    const std::size_t insertionDetailCount = std::min<std::size_t>(8, slowInsertions.size());
    for (std::size_t i = 0; i < insertionDetailCount; ++i) {
        const BowyerInsertionStats& insertion = slowInsertions[i];
        output << "[navmesh-cdt] region=" << regionIndex << " slow_point_rank=" << (i + 1)
            << " point_index=" << insertion.pointIndex
            << " xy=(" << insertion.point.x << ',' << insertion.point.y << ')'
            << " elapsed_ms=" << insertion.elapsedMs
            << " stored_triangle_slots=" << insertion.storedTriangleSlots
            << " live_triangles_scanned=" << insertion.liveTrianglesScanned
            << " cavity_triangles=" << insertion.cavityTriangles
            << " boundary_edges=" << insertion.boundaryEdges
            << " triangles_added=" << insertion.trianglesAdded << '\n';
    }

    std::vector<ConstraintRecoveryStats> expensiveSegments = diagnostics.recoverySegments;
    std::stable_sort(expensiveSegments.begin(), expensiveSegments.end(), [](const auto& a, const auto& b) {
        return a.elapsedMs > b.elapsedMs;
    });
    std::vector<ConstraintRecoveryStats> selectedSegments;
    selectedSegments.reserve(expensiveSegments.size());
    for (const ConstraintRecoveryStats& segment : expensiveSegments)
        if (segment.cavityAttempts > 0) selectedSegments.push_back(segment);
    std::size_t additionalSlowSegmentCount = 0;
    for (const ConstraintRecoveryStats& segment : expensiveSegments) {
        if (segment.cavityAttempts > 0 || additionalSlowSegmentCount >= 12) continue;
        selectedSegments.push_back(segment);
        ++additionalSlowSegmentCount;
    }
    for (std::size_t i = 0; i < selectedSegments.size(); ++i) {
        const ConstraintRecoveryStats& segment = selectedSegments[i];
        output << "[navmesh-cdt] region=" << regionIndex << " slow_constraint_rank=" << (i + 1)
            << " contour_edge=" << segment.constraintIndex
            << " original_edge=(" << segment.originalConstraint[0] << ',' << segment.originalConstraint[1] << ')'
            << " piece=" << (segment.segmentIndex + 1) << '/' << segment.segmentCount
            << " endpoint_indices=(" << segment.segment[0] << ',' << segment.segment[1] << ')'
            << " xy=(" << segment.segmentStart.x << ',' << segment.segmentStart.y << " -> "
            << segment.segmentEnd.x << ',' << segment.segmentEnd.y << ')'
            << " point_on_segment_checks=" << segment.pointChecks
            << " elapsed_ms=" << segment.elapsedMs
            << " outcome=" << segment.outcome
            << " iterations=" << segment.iterations << '/' << segment.iterationLimit
            << " edge_entries_scanned=" << segment.edgeEntriesScanned
            << " intersection_tests=" << segment.intersectionTests
            << " crossing_edges_seen=" << segment.crossingEdgesSeen
            << " flip_attempts=" << segment.flipAttempts
            << " successful_flips=" << segment.successfulFlips
            << " progress_flips=" << segment.progressFlips
            << " non_progress_flips=" << segment.nonProgressFlips
            << " max_non_progress_streak=" << segment.maximumNonProgressFlipStreak
            << " early_cavity_attempted=" << (segment.earlyCavityAttempted ? "true" : "false")
            << " early_cavity_recovered=" << (segment.earlyCavityRecovered ? "true" : "false")
            << " cavity_attempts=" << segment.cavityAttempts
            << " cavity_ms=" << segment.cavityMs
            << " cavity_edge_entries_scanned=" << segment.cavityEdgeEntriesScanned
            << " cavity_crossed_edges=" << segment.cavityCrossedEdges
            << " cavity_removed_triangles=" << segment.cavityRemovedTriangles
            << " cavity_boundary_vertices=" << segment.cavityBoundaryVertices
            << " cavity_fill_triangles=" << segment.cavityFillTriangles
            << " cavity_path_ms=" << segment.cavityPathMs;
        if (!segment.cavityFailure.empty()) output << " cavity_failure=" << segment.cavityFailure;
        output << '\n';
    }

    std::clog << output.str();
}

/// Keep faces whose centers fall inside the outer contour and outside every hole.
/// @param input Region contours and the point coordinates they refer to.
/// @param triangles Candidate faces from the constrained triangulation.
/// @param mesh Output mesh receiving the selected faces; its existing vertices are preserved.
/// @return No value. Faces are appended as traversable triangles.
void KeepTrianglesInsideRegion(const InputGeometry& input, const std::vector<Triangle>& triangles,
    ConstrainedMesh& mesh) {
    // Use the average of the three corners as one representative point for each triangle.
    for (const Triangle& triangle : triangles) {
        const Vector2d triangleCenter = (input.points[triangle[0]] + input.points[triangle[1]] +
            input.points[triangle[2]]) / 3.0;
        // The first contour is the outside boundary; a center outside it means the whole face is unwanted.
        if (!PointInLoop(triangleCenter, input.contours.front(), input.points)) continue;
        /// Check each later contour (a hole); a center inside any one hole must be discarded.
        const bool insideHole = [&] {
            for (std::size_t contourIndex = 1; contourIndex < input.contours.size(); ++contourIndex)
                if (PointInLoop(triangleCenter, input.contours[contourIndex], input.points)) return true;
            return false;
        }();
        if (insideHole) continue;
        mesh.triangles.push_back({triangle, {}, TriangleRegion::Traversable});
    }
}

/// Check that the filtered mesh is non-empty and every selected vertex is used by a face.
/// @param mesh Mesh to inspect after region filtering.
/// @return True only when it has at least one triangle and no unused vertex.
bool UsesEveryInputVertex(const ConstrainedMesh& mesh) {
    // Mark each referenced vertex, then require a non-empty mesh with no unused vertices.
    // Begin: record vertex use from all retained triangles.
    std::vector<bool> usedVertices(mesh.vertices.size(), false);
    for (const MeshTriangle& triangle : mesh.triangles)
        for (std::size_t vertex : triangle.vertices) usedVertices[vertex] = true;
    // End: accept only a non-empty mesh that uses every selected input vertex.
    return !mesh.triangles.empty() && std::find(usedVertices.begin(), usedVertices.end(), false) == usedVertices.end();
}

/// Append a region mesh to a combined mesh, adjusting its local indices to the combined vertex list.
/// @param input Region mesh and its local vertex numbering.
/// @param constraints That region's contour edges, also using local indices.
/// @param combinedMesh Destination mesh; new vertices, triangles, and constraints are appended.
/// @return No value. Every copied index is shifted by the destination's old vertex count.
void AppendRegionMesh(const InputGeometry& input, const std::set<Edge>& constraints,
    ConstrainedMesh& combinedMesh) {
    // The region starts numbering vertices at zero; move all its indices past vertices already appended.
    const std::size_t baseVertexIndex = combinedMesh.vertices.size();
    combinedMesh.vertices.insert(combinedMesh.vertices.end(), input.mesh.vertices.begin(), input.mesh.vertices.end());
    for (MeshTriangle triangle : input.mesh.triangles) {
        // Update the copy, not the region's original triangle.
        for (std::size_t& vertexIndex : triangle.vertices) vertexIndex += baseVertexIndex;
        combinedMesh.triangles.push_back(triangle);
    }
    for (const Edge& edge : constraints)
        combinedMesh.constraints.push_back({edge[0] + baseVertexIndex, edge[1] + baseVertexIndex});
}

} // namespace

/// Triangulate prepared walkable regions and publish a NavMesh only after the combined topology is valid.
/// @param regions Sanitized outer contours and hole contours ready for triangulation.
/// @return Success with an immutable NavMesh, or failure with a user-facing explanation and no mesh.
GenerationResult ConstrainedDelaunay::Triangulate(const detail::WalkableRegionSet& regions) const {
    const bool diagnosticsEnabled = NavmeshDiagnosticsEnabled();
    const auto started = diagnosticsEnabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    std::string error;
    ConstrainedMesh combined;
    std::size_t regionCount = 0;
    std::size_t regionIndex = 0;
    for (const detail::WalkableRegion& region : regions.Regions()) {
        const auto regionStarted = diagnosticsEnabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
        const auto inputAssemblyStarted = diagnosticsEnabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
        CdtDiagnostics diagnostics(diagnosticsEnabled);
        InputGeometry input;
        // Offset vertices have no source identity unless the builder preserved an exact source match.
        /// Copy one contour into this region and keep its point, vertex, and contour indices aligned.
        /// @param contour Ordered coordinates around the outside or one hole.
        /// @param sourceIds Optional source IDs in the same order; each entry belongs to the same-position vertex.
        /// @return No value. A contour with at least three points is added to the region's contour list.
        auto addContour = [&](const std::vector<Vector2d>& contour,
            const std::vector<std::optional<std::uint64_t>>& sourceIds) {
            // Each region uses local indices shared by its point array, output vertices, and contours.
            ContourIndices contourIndices;
            contourIndices.vertices.reserve(contour.size());
            for (std::size_t i = 0; i < contour.size(); ++i) {
                const Vector2d& position = contour[i];
                // The index is assigned once and reused by the triangulator and output mesh for this point.
                const std::size_t index = input.points.size();
                input.points.push_back(position);
                input.mesh.vertices.push_back({sourceIds[i], position});
                contourIndices.vertices.push_back(index);
            }
            if (contourIndices.vertices.size() >= 3) input.contours.push_back(std::move(contourIndices));
        };
        addContour(region.outerContour, region.outerSourceIds);
        for (std::size_t i = 0; i < region.holeContours.size(); ++i)
            addContour(region.holeContours[i], region.holeSourceIds[i]);
        std::set<Edge> constraints;
        BuildConstraints(input, constraints);
        const double inputAssemblyMs = diagnosticsEnabled ? ElapsedMilliseconds(inputAssemblyStarted) : 0.0;
        std::vector<Triangle> triangles;
        // Delaunay seed, constraint recovery, then unconstrained-edge legalization form one failure-aware stage.
        if (!BuildConstrainedTriangulation(input, constraints, triangles, error, &diagnostics)) {
            PrintCdtRegionDiagnostics(regionIndex, input, input.contours.size(), constraints.size(), diagnostics,
                inputAssemblyMs, 0.0, 0.0, diagnosticsEnabled ? ElapsedMilliseconds(regionStarted) : 0.0,
                0, "failed-build");
            return {GenerationStatus::Failed, {}, "Could not triangulate offset contours: " + error};
        }
        const auto filterStarted = diagnosticsEnabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
        KeepTrianglesInsideRegion(input, triangles, input.mesh);
        const bool usesEveryInputVertex = UsesEveryInputVertex(input.mesh);
        const double filterMs = diagnosticsEnabled ? ElapsedMilliseconds(filterStarted) : 0.0;
        if (!usesEveryInputVertex) {
            PrintCdtRegionDiagnostics(regionIndex, input, input.contours.size(), constraints.size(), diagnostics,
                inputAssemblyMs, filterMs, 0.0, diagnosticsEnabled ? ElapsedMilliseconds(regionStarted) : 0.0,
                input.mesh.triangles.size(), "failed-unused-input-vertex");
            return {GenerationStatus::Failed, {}, "An offset walkable component was not completely triangulated."};
        }

        const auto appendStarted = diagnosticsEnabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
        AppendRegionMesh(input, constraints, combined);
        const double appendMs = diagnosticsEnabled ? ElapsedMilliseconds(appendStarted) : 0.0;
        ++regionCount;
        PrintCdtRegionDiagnostics(regionIndex, input, input.contours.size(), constraints.size(), diagnostics,
            inputAssemblyMs, filterMs, appendMs, diagnosticsEnabled ? ElapsedMilliseconds(regionStarted) : 0.0,
            input.mesh.triangles.size(), "success");
        ++regionIndex;
    }

    if (regionCount == 0 || combined.triangles.empty()) {
        if (diagnosticsEnabled)
            std::clog << "[navmesh-cdt] result=failed stage=empty-combined-mesh regions=" << regionCount
                << " vertices=" << combined.vertices.size() << " triangles=" << combined.triangles.size()
                << " total_ms=" << ElapsedMilliseconds(started) << '\n';
        return {GenerationStatus::Failed, {}, "Agent radius leaves no triangulatable walkable area."};
    }
    // This final pass checks reciprocal links and manifold edge use; only then is the draft published.
    const auto adjacencyStarted = diagnosticsEnabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
    const bool adjacencyBuilt = BuildFinalAdjacency(combined);
    const double adjacencyMs = diagnosticsEnabled ? ElapsedMilliseconds(adjacencyStarted) : 0.0;
    if (!adjacencyBuilt) {
        if (diagnosticsEnabled)
            std::clog << "[navmesh-cdt] result=failed stage=final-adjacency elapsed_ms=" << adjacencyMs
                << " regions=" << regionCount << " vertices=" << combined.vertices.size()
                << " triangles=" << combined.triangles.size() << " total_ms=" << ElapsedMilliseconds(started) << '\n';
        return {GenerationStatus::Failed, {}, "Generated navmesh has non-manifold or invalid adjacency."};
    }
    if (diagnosticsEnabled)
        std::clog << "[navmesh-cdt] result=success regions=" << regionCount
            << " vertices=" << combined.vertices.size() << " triangles=" << combined.triangles.size()
            << " constraints=" << combined.constraints.size() << " final_adjacency_ms=" << adjacencyMs
            << " total_ms=" << ElapsedMilliseconds(started) << '\n';
    return {GenerationStatus::Success, NavMesh(std::move(combined)), "Navmesh baked successfully."};
}

} // namespace cdt
