#include "cdt/ConstrainedDelaunay.hpp"
#include "core/GeometryPredicates.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <optional>
#include <set>
#include <sstream>
#include <string>


namespace cdt {

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
// Store the smaller index first so undirected edges have one canonical key.
Edge MakeEdge(std::size_t firstVertexIndex, std::size_t secondVertexIndex) {
    return firstVertexIndex < secondVertexIndex ? Edge{firstVertexIndex, secondVertexIndex} : Edge{secondVertexIndex, firstVertexIndex};
}

// Add a face and update its edge records and neighboring-face links.
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

// Remove a face from edge adjacency and clear neighbor links on the remaining faces.
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

// Remembers the first triangle and edge slot that use a particular mesh edge.
struct FirstEdgeUse {
    std::size_t triangleIndex{};
    std::size_t edgeSlot{};
};

// Fill each triangle's neighbor slots from the edges in the final, filtered mesh.
// A slot points across the edge opposite the vertex with the same index.
bool BuildFinalAdjacency(ConstrainedMesh& mesh) {
    std::map<Edge, FirstEdgeUse> firstTriangleByEdge;

    // Clear old links and process every edge of every final triangle.
    for (std::size_t triangleIndex = 0; triangleIndex < mesh.triangles.size(); ++triangleIndex) {
        MeshTriangle& triangle = mesh.triangles[triangleIndex];
        triangle.neighbors = {};

        for (std::size_t edgeSlot = 0; edgeSlot < 3; ++edgeSlot) {
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

// Report whether the mesh currently contains the requested undirected edge.
bool HasEdge(const TriangleMesh& triangleMesh, const Edge& edge) { return triangleMesh.trianglesByEdge.contains(edge); }

// Check the raw Bowyer-Watson faces before they are copied into the adjacency map.
bool TrianglesContainEdge(const Edge& edge, const std::vector<Triangle>& triangles) {
    for (const Triangle& triangle : triangles)
        for (std::size_t i = 0; i < triangle.size(); ++i)
            if (MakeEdge(triangle[i], triangle[(i + 1) % triangle.size()]) == edge) return true;
    return false;
}

// Print the raw Bowyer-Watson faces incident to either endpoint, including their vertex coordinates.
std::string DescribeRawEndpointTriangles(const Edge& constraint, const std::vector<Triangle>& triangles,
    const std::vector<Vector2d>& points) {
    constexpr std::size_t maxPrintedTriangles = 64;
    std::ostringstream description;
    description << std::setprecision(17);
    std::size_t touchingTriangleCount = 0;
    std::size_t printedTriangleCount = 0;
    for (std::size_t triangleIndex = 0; triangleIndex < triangles.size(); ++triangleIndex) {
        const Triangle& triangle = triangles[triangleIndex];
        const bool touchesEndpoint = std::find(triangle.begin(), triangle.end(), constraint[0]) != triangle.end() ||
            std::find(triangle.begin(), triangle.end(), constraint[1]) != triangle.end();
        if (!touchesEndpoint) continue;
        ++touchingTriangleCount;
        if (printedTriangleCount == maxPrintedTriangles) continue;
        if (printedTriangleCount != 0) description << "; ";
        description << "t" << triangleIndex << "=[";
        for (std::size_t vertexSlot = 0; vertexSlot < triangle.size(); ++vertexSlot) {
            if (vertexSlot != 0) description << ", ";
            const std::size_t vertex = triangle[vertexSlot];
            description << vertex << "=(" << points[vertex].x << "," << points[vertex].y << ")";
        }
        description << "]";
        ++printedTriangleCount;
    }
    if (touchingTriangleCount == 0) return "(none)";
    if (touchingTriangleCount > printedTriangleCount)
        description << "; ... " << (touchingTriangleCount - printedTriangleCount) << " more";
    return description.str();
}

// Return the signed side of C relative to AB. Other functions use the sign to test turns and crossings.
double Orient(const Vector2d& firstPoint, const Vector2d& secondPoint, const Vector2d& thirdPoint) {
    return geometry::Orient2D(firstPoint, secondPoint, thirdPoint);
}

// Return the signed circle test for D against ABC. The triangulation uses it to choose triangles to replace.
double InBall(const Vector2d& firstTrianglePoint, const Vector2d& secondTrianglePoint,
    const Vector2d& thirdTrianglePoint, const Vector2d& testPoint) {
    return geometry::InCircle2D(firstTrianglePoint, secondTrianglePoint, thirdTrianglePoint, testPoint);
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

// Test contour membership with the shared ray-cast predicate; boundary points are excluded so
// region filtering follows a consistent strict-interior policy.
bool PointInLoop(const Vector2d& point, const ContourIndices& contour, const std::vector<Vector2d>& points) {
    std::vector<Vector2d> polygon;
    polygon.reserve(contour.vertices.size());
    for (std::size_t index : contour.vertices) polygon.push_back(points[index]);
    return geometry::PointInPolygon(polygon, point, geometry::BoundaryPolicy::Outside);
}

// Compare which side each segment's endpoints lie on. Opposite sides on both segments mean they cross inside.
bool ProperlyIntersects(const Vector2d& a, const Vector2d& b, const Vector2d& c, const Vector2d& d) {
    const double ab_c = Orient(a, b, c), ab_d = Orient(a, b, d);
    const double cd_a = Orient(c, d, a), cd_b = Orient(c, d, b);
    return ((ab_c > 0.0 && ab_d < 0.0) || (ab_c < 0.0 && ab_d > 0.0)) &&
        ((cd_a > 0.0 && cd_b < 0.0) || (cd_a < 0.0 && cd_b > 0.0));
}

std::size_t CountProperCrossingEdges(const Edge& segment, const TriangleMesh& triangleMesh,
    const std::vector<Vector2d>& points) {
    std::size_t count = 0;
    for (const auto& [edge, uses] : triangleMesh.trianglesByEdge) {
        (void)uses;
        if (ProperlyIntersects(points[segment[0]], points[segment[1]], points[edge[0]], points[edge[1]])) ++count;
    }
    return count;
}

std::size_t CountIncidentEdges(std::size_t vertex, const TriangleMesh& triangleMesh) {
    std::size_t count = 0;
    for (const auto& [edge, uses] : triangleMesh.trianglesByEdge) {
        (void)uses;
        if (edge[0] == vertex || edge[1] == vertex) ++count;
    }
    return count;
}

std::string DescribeConstraintNeighborhood(const Edge& constraint, const TriangleMesh& triangleMesh,
    const std::vector<Vector2d>& points) {
    std::ostringstream description;
    description << std::setprecision(17);
    for (std::size_t endpointSlot = 0; endpointSlot < constraint.size(); ++endpointSlot) {
        if (endpointSlot != 0) description << "; ";
        const std::size_t endpoint = constraint[endpointSlot];
        description << "endpoint " << endpoint << "=(" << points[endpoint].x << "," << points[endpoint].y
            << ") adjacent vertices [";
        bool firstNeighbor = true;
        for (const auto& [edge, uses] : triangleMesh.trianglesByEdge) {
            (void)uses;
            if (edge[0] != endpoint && edge[1] != endpoint) continue;
            const std::size_t neighbor = edge[0] == endpoint ? edge[1] : edge[0];
            if (!firstNeighbor) description << ", ";
            firstNeighbor = false;
            description << neighbor << "=(" << points[neighbor].x << "," << points[neighbor].y
                << "; orient=" << Orient(points[constraint[0]], points[constraint[1]], points[neighbor]) << ")";
        }
        description << "]";
    }
    return description.str();
}

bool OnSegment(const Vector2d& a, const Vector2d& b, const Vector2d& p) {
    return Orient(a, b, p) == 0.0 && p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x) &&
        p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y);
}

double PolygonArea2(const std::vector<std::size_t>& polygon, const std::vector<Vector2d>& points) {
    std::vector<Vector2d> positions;
    positions.reserve(polygon.size());
    for (std::size_t vertex : polygon) positions.push_back(points[vertex]);
    return geometry::SignedTwiceArea(positions);
}

bool PointInTriangleInclusive(const Vector2d& p, const Vector2d& a, const Vector2d& b, const Vector2d& c) {
    const double ab = Orient(a, b, p), bc = Orient(b, c, p), ca = Orient(c, a, p);
    return (ab >= 0.0 && bc >= 0.0 && ca >= 0.0) || (ab <= 0.0 && bc <= 0.0 && ca <= 0.0);
}

// Ear clip a simple polygon. Collinear boundary vertices are retained; they are
// skipped as ears but remain represented by adjacent non-degenerate faces.
bool TriangulateSimplePolygon(std::vector<std::size_t> polygon, const std::vector<Vector2d>& points,
    std::vector<Triangle>& result, std::string& failureReason) {
    if (polygon.size() < 3) {
        failureReason = "cavity side has fewer than three boundary vertices";
        return false;
    }
    const double area2 = PolygonArea2(polygon, points);
    if (area2 == 0.0) {
        failureReason = "cavity side has zero signed area";
        return false;
    }
    if (area2 < 0.0) std::reverse(polygon.begin(), polygon.end());
    while (polygon.size() > 3) {
        bool clipped = false;
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            const auto prev = polygon[(i + polygon.size() - 1) % polygon.size()];
            const auto curr = polygon[i];
            const auto next = polygon[(i + 1) % polygon.size()];
            if (Orient(points[prev], points[curr], points[next]) <= 0.0) continue;
            bool containsOther = false;
            for (std::size_t j = 0; j < polygon.size(); ++j) {
                const auto candidate = polygon[j];
                if (candidate == prev || candidate == curr || candidate == next) continue;
                if (PointInTriangleInclusive(points[candidate], points[prev], points[curr], points[next])) {
                    containsOther = true; break;
                }
            }
            if (containsOther) continue;
            result.push_back({prev, curr, next});
            polygon.erase(polygon.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true; break;
        }
        if (!clipped) {
            failureReason = "ear clipping found no valid ear with " + std::to_string(polygon.size()) + " vertices remaining";
            return false;
        }
    }
    if (Orient(points[polygon[0]], points[polygon[1]], points[polygon[2]]) == 0.0) {
        failureReason = "ear clipping left a zero-area final triangle";
        return false;
    }
    result.push_back(Ccw({polygon[0], polygon[1], polygon[2]}, points));
    return true;
}

bool RecoverByCavity(const Edge& constraint, TriangleMesh& triangleMesh, const std::vector<Vector2d>& points,
    const std::set<Edge>& fixedConstraintEdges, std::string& failureReason) {
    std::vector<Edge> crossedEdges;
    for (const auto& [edge, uses] : triangleMesh.trianglesByEdge) {
        (void)uses;
        if (ProperlyIntersects(points[constraint[0]], points[constraint[1]], points[edge[0]], points[edge[1]])) {
            if (fixedConstraintEdges.contains(edge)) {
                failureReason = "requested segment crosses previously recovered constraint edge [" +
                    std::to_string(edge[0]) + ", " + std::to_string(edge[1]) + "]";
                return false;
            }
            crossedEdges.push_back(edge);
        }
    }
    if (crossedEdges.empty()) {
        failureReason = "edge flipping stalled, but no mesh edge properly crosses the requested segment";
        return false;
    }
    std::set<TriangleIndex> removed;
    for (const Edge& edge : crossedEdges) {
        const auto it = triangleMesh.trianglesByEdge.find(edge);
        if (it == triangleMesh.trianglesByEdge.end() || it->second.triangleCount != 2) {
            failureReason = "crossed edge [" + std::to_string(edge[0]) + ", " + std::to_string(edge[1]) +
                "] is missing or does not have exactly two adjacent triangles";
            return false;
        }
        for (std::size_t i = 0; i < 2; ++i) removed.insert(it->second.triangles[i].triangleIndex);
    }

    std::map<Edge, int> counts;
    for (TriangleIndex index : removed) {
        const Triangle& t = triangleMesh.triangles[index].vertexIndices;
        for (std::size_t i = 0; i < 3; ++i) ++counts[MakeEdge(t[i], t[(i + 1) % 3])];
    }
    std::map<std::size_t, std::vector<std::size_t>> graph;
    for (const auto& [edge, count] : counts) if (count == 1) {
        graph[edge[0]].push_back(edge[1]); graph[edge[1]].push_back(edge[0]);
    }
    if (graph[constraint[0]].size() != 2 || graph[constraint[1]].size() != 2) {
        failureReason = "cavity boundary does not have two paths at the requested endpoints (degrees " +
            std::to_string(graph[constraint[0]].size()) + " and " +
            std::to_string(graph[constraint[1]].size()) + ")";
        return false;
    }

    auto makePath = [&](std::size_t firstNeighbor, std::string& pathFailure) -> std::vector<std::size_t> {
        std::vector<std::size_t> path{constraint[0]};
        std::size_t previous = constraint[0], current = firstNeighbor;
        while (true) {
            if (std::find(path.begin(), path.end(), current) != path.end()) {
                pathFailure = "cavity boundary walk loops before reaching the other endpoint";
                return {};
            }
            path.push_back(current);
            if (current == constraint[1]) return path;
            const auto it = graph.find(current);
            if (it == graph.end() || it->second.size() != 2) {
                pathFailure = "cavity boundary walk reached a vertex whose degree is not two";
                return {};
            }
            if (path.size() > graph.size()) {
                pathFailure = "cavity boundary walk exceeded the number of boundary vertices";
                return {};
            }
            const std::size_t next = it->second[0] == previous ? it->second[1] : it->second[0];
            previous = current; current = next;
        }
    };
    std::string pathFailure;
    auto first = makePath(graph[constraint[0]][0], pathFailure);
    if (first.empty()) {
        failureReason = "first cavity boundary path failed: " + pathFailure;
        return false;
    }
    auto second = makePath(graph[constraint[0]][1], pathFailure);
    if (second.empty()) {
        failureReason = "second cavity boundary path failed: " + pathFailure;
        return false;
    }
    if (first == second) {
        failureReason = "both cavity boundary walks produced the same path";
        return false;
    }

    std::vector<Triangle> fill;
    std::string triangulationFailure;
    if (!TriangulateSimplePolygon(std::move(first), points, fill, triangulationFailure)) {
        failureReason = "could not triangulate first cavity side: " + triangulationFailure;
        return false;
    }
    if (!TriangulateSimplePolygon(std::move(second), points, fill, triangulationFailure)) {
        failureReason = "could not triangulate second cavity side: " + triangulationFailure;
        return false;
    }
    TriangleMesh candidate = triangleMesh;
    for (TriangleIndex index : removed) RemoveTriangle(candidate, index);
    if (!candidate.hasValidTopology) {
        failureReason = "removing the crossed strip left invalid mesh topology";
        return false;
    }
    for (const Triangle& t : fill) {
        if (Orient(points[t[0]], points[t[1]], points[t[2]]) == 0.0) {
            failureReason = "cavity triangulation produced a zero-area face";
            return false;
        }
        if (!AddTriangle(candidate, t)) {
            failureReason = "adding a cavity face would create non-manifold topology";
            return false;
        }
    }
    if (!candidate.hasValidTopology) {
        failureReason = "candidate cavity mesh has invalid topology after retriangulation";
        return false;
    }
    if (!HasEdge(candidate, constraint)) {
        failureReason = "candidate cavity mesh does not contain the requested constraint edge";
        return false;
    }
    for (const Edge& fixed : fixedConstraintEdges) if (!HasEdge(candidate, fixed)) {
        failureReason = "candidate cavity mesh would remove previously recovered constraint edge [" +
            std::to_string(fixed[0]) + ", " + std::to_string(fixed[1]) + "]";
        return false;
    }
    triangleMesh = std::move(candidate);
    return true;
}

bool FlipEdge(TriangleMesh& triangleMesh, const Edge& edge, const std::vector<Vector2d>& points);

bool RecoverConstraintSingle(const Edge& constraint, TriangleMesh& triangleMesh,
    const std::vector<Vector2d>& points, std::set<Edge>& fixedConstraintEdges, std::string& failureReason) {
    const std::size_t iterationLimit = triangleMesh.trianglesByEdge.size() * triangleMesh.trianglesByEdge.size() + 32;
    for (std::size_t iteration = 0; iteration < iterationLimit; ++iteration) {
        if (HasEdge(triangleMesh, constraint)) { fixedConstraintEdges.insert(constraint); return true; }
        std::vector<Edge> crossingEdges;
        for (const auto& [edge, trianglesOnEdge] : triangleMesh.trianglesByEdge) {
            (void)trianglesOnEdge;
            if (edge != constraint && ProperlyIntersects(points[constraint[0]], points[constraint[1]], points[edge[0]], points[edge[1]])) crossingEdges.push_back(edge);
        }
        bool flipped = false;
        for (const Edge& edge : crossingEdges) if (!fixedConstraintEdges.contains(edge) && FlipEdge(triangleMesh, edge, points)) {
            flipped = true; break;
        }
        if (!flipped) {
            if (RecoverByCavity(constraint, triangleMesh, points, fixedConstraintEdges, failureReason))
                return (fixedConstraintEdges.insert(constraint), true);
            failureReason = "edge flipping stalled; cavity recovery failed: " + failureReason;
            return false;
        }
    }
    if (RecoverByCavity(constraint, triangleMesh, points, fixedConstraintEdges, failureReason))
        return (fixedConstraintEdges.insert(constraint), true);
    failureReason = "edge flipping reached its iteration limit; cavity recovery failed: " + failureReason;
    return false;
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

// Build the convex hull in counterclockwise order, retaining every input point on its boundary.
// The monotone-chain pass finds strict corners; points on each hull edge are then inserted in edge order.
bool BuildConvexHullBoundary(const std::vector<Vector2d>& points, std::vector<std::size_t>& hull,
    std::string& failureReason) {
    hull.clear();
    if (points.size() < 3) {
        failureReason = "fewer than three input points";
        return false;
    }

    std::vector<std::size_t> sorted(points.size());
    for (std::size_t i = 0; i < sorted.size(); ++i) sorted[i] = i;
    std::sort(sorted.begin(), sorted.end(), [&](std::size_t a, std::size_t b) {
        if (points[a].x != points[b].x) return points[a].x < points[b].x;
        if (points[a].y != points[b].y) return points[a].y < points[b].y;
        return a < b;
    });
    for (std::size_t i = 1; i < sorted.size(); ++i) {
        const Vector2d& previous = points[sorted[i - 1]];
        const Vector2d& current = points[sorted[i]];
        if (previous.x == current.x && previous.y == current.y) {
            failureReason = "input contains coincident points";
            return false;
        }
    }

    std::vector<std::size_t> lower, upper;
    lower.reserve(sorted.size());
    upper.reserve(sorted.size());
    for (std::size_t index : sorted) {
        while (lower.size() >= 2 &&
            Orient(points[lower[lower.size() - 2]], points[lower.back()], points[index]) <= 0.0)
            lower.pop_back();
        lower.push_back(index);
    }
    for (auto it = sorted.rbegin(); it != sorted.rend(); ++it) {
        const std::size_t index = *it;
        while (upper.size() >= 2 &&
            Orient(points[upper[upper.size() - 2]], points[upper.back()], points[index]) <= 0.0)
            upper.pop_back();
        upper.push_back(index);
    }
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    if (lower.size() < 3) {
        failureReason = "all input points are collinear";
        return false;
    }

    hull.reserve(points.size());
    for (std::size_t edgeIndex = 0; edgeIndex < lower.size(); ++edgeIndex) {
        const std::size_t first = lower[edgeIndex];
        const std::size_t second = lower[(edgeIndex + 1) % lower.size()];
        hull.push_back(first);

        std::vector<std::size_t> edgePoints;
        for (std::size_t candidate = 0; candidate < points.size(); ++candidate) {
            if (candidate != first && candidate != second &&
                OnSegment(points[first], points[second], points[candidate]))
                edgePoints.push_back(candidate);
        }
        const Vector2d direction = points[second] - points[first];
        const bool sortByX = std::abs(direction.x) >= std::abs(direction.y);
        const bool ascending = sortByX ? direction.x > 0.0 : direction.y > 0.0;
        std::sort(edgePoints.begin(), edgePoints.end(), [&](std::size_t a, std::size_t b) {
            const double coordinateA = sortByX ? points[a].x : points[a].y;
            const double coordinateB = sortByX ? points[b].x : points[b].y;
            return ascending ? coordinateA < coordinateB : coordinateA > coordinateB;
        });
        hull.insert(hull.end(), edgePoints.begin(), edgePoints.end());
    }
    return true;
}

bool Legalize(TriangleMesh& triangleMesh, const std::vector<Vector2d>& points,
    const std::set<Edge>& fixedConstraintEdges);

// Add interior points one at a time, replacing triangles whose circles contain each point.
// The real convex hull remains the mesh boundary throughout cavity removal.
std::vector<Triangle> BowyerWatson(const std::vector<Vector2d>& input, std::vector<Vector2d>& points,
    std::string& diagnosticTrace) {
    std::ostringstream trace;
    trace << std::setprecision(17);
    points = input;
    trace << "Bowyer-Watson input points (index: x,y), count=" << points.size() << ":\n";
    for (std::size_t pointIndex = 0; pointIndex < points.size(); ++pointIndex)
        trace << "  p" << pointIndex << "=(" << points[pointIndex].x << "," << points[pointIndex].y << ")\n";
    if (points.size() < 3) {
        trace << "RETURN: fewer than three input points\n";
        diagnosticTrace = trace.str();
        return {};
    }
    double minimumX = points[0].x, maximumX = minimumX, minimumY = points[0].y, maximumY = minimumY;
    for (const Vector2d& p : points) {
        minimumX = std::min(minimumX, p.x); maximumX = std::max(maximumX, p.x);
        minimumY = std::min(minimumY, p.y); maximumY = std::max(maximumY, p.y);
    }
    const double boundingBoxSpan = std::max(maximumX - minimumX, maximumY - minimumY);
    trace << "input bounds: min=(" << minimumX << "," << minimumY << ") max=(" << maximumX << "," << maximumY
        << ") span=" << boundingBoxSpan << "\n";
    if (!std::isfinite(boundingBoxSpan) || boundingBoxSpan <= 0.0) {
        trace << "RETURN: invalid or zero bounding-box span\n";
        diagnosticTrace = trace.str();
        return {};
    }
    const std::size_t n = points.size();
    TriangleMesh triangleMesh;

    auto writeTriangle = [&](std::size_t triangleIndex, const Triangle& triangle) {
        trace << "t" << triangleIndex << "=[";
        for (std::size_t vertexSlot = 0; vertexSlot < triangle.size(); ++vertexSlot) {
            if (vertexSlot != 0) trace << ", ";
            const std::size_t vertex = triangle[vertexSlot];
            trace << vertex << "=(" << points[vertex].x << "," << points[vertex].y << ")";
        }
        trace << "]";
    };
    auto writeLiveMesh = [&](const char* label) {
        trace << label << " (hasValidTopology=" << (triangleMesh.hasValidTopology ? "true" : "false") << "):\n";
        for (std::size_t triangleIndex = 0; triangleIndex < triangleMesh.triangles.size(); ++triangleIndex) {
            const StoredTriangle& stored = triangleMesh.triangles[triangleIndex];
            if (!stored.isPresent) continue;
            trace << "  ";
            writeTriangle(triangleIndex, stored.vertexIndices);
            trace << "\n";
        }
        trace << "  live edge records:\n";
        for (const auto& [edge, uses] : triangleMesh.trianglesByEdge) {
            trace << "    edge " << edge[0] << "=(" << points[edge[0]].x << "," << points[edge[0]].y << ") -- "
                << edge[1] << "=(" << points[edge[1]].x << "," << points[edge[1]].y << ") uses="
                << uses.triangleCount << " triangles=";
            for (std::size_t useIndex = 0; useIndex < uses.triangleCount; ++useIndex) {
                if (useIndex != 0) trace << ",";
                trace << uses.triangles[useIndex].triangleIndex;
            }
            trace << "\n";
        }
    };

    std::vector<std::size_t> hull;
    std::string hullFailureReason;
    if (!BuildConvexHullBoundary(points, hull, hullFailureReason)) {
        trace << "RETURN: could not build real convex hull: " << hullFailureReason << "\n";
        diagnosticTrace = trace.str();
        return {};
    }
    trace << "real convex hull boundary (counterclockwise, index: x,y), count=" << hull.size() << ":\n";
    for (std::size_t vertex : hull)
        trace << "  p" << vertex << "=(" << points[vertex].x << "," << points[vertex].y << ")\n";

    std::vector<Triangle> hullTriangles;
    std::string hullTriangulationFailure;
    if (!TriangulateSimplePolygon(hull, points, hullTriangles, hullTriangulationFailure)) {
        trace << "RETURN: could not triangulate real convex hull: " << hullTriangulationFailure << "\n";
        diagnosticTrace = trace.str();
        return {};
    }
    trace << "real hull seed triangles (count=" << hullTriangles.size() << "):\n";
    for (std::size_t triangleIndex = 0; triangleIndex < hullTriangles.size(); ++triangleIndex) {
        trace << "  ";
        writeTriangle(triangleIndex, hullTriangles[triangleIndex]);
        trace << "\n";
        if (!AddTriangle(triangleMesh, hullTriangles[triangleIndex])) {
            trace << "RETURN: hull seed faces could not form valid adjacency\n";
            diagnosticTrace = trace.str();
            return {};
        }
    }
    const std::set<Edge> noFixedConstraints;
    if (!Legalize(triangleMesh, points, noFixedConstraints)) {
        trace << "RETURN: real hull seed legalization did not converge\n";
        diagnosticTrace = trace.str();
        return {};
    }
    std::vector<bool> isHullVertex(n, false);
    for (std::size_t vertex : hull) isHullVertex[vertex] = true;
    writeLiveMesh("mesh after real hull seed legalization and before interior insertions");

    // Point insertion: remove triangles whose circumcircles contain the point, then fill the cavity
    // from edges that occur only once on the removed triangles' boundary.
    for (std::size_t pointIndex = 0; pointIndex < n; ++pointIndex) {
        if (isHullVertex[pointIndex]) {
            trace << "\nSKIP p" << pointIndex << " because it is already in the real hull seed\n";
            continue;
        }
        trace << "\nINSERT p" << pointIndex << "=(" << points[pointIndex].x << "," << points[pointIndex].y
            << ") stored_triangle_slots=" << triangleMesh.triangles.size() << "\n";
        std::vector<bool> trianglesToRemove(triangleMesh.triangles.size(), false);
        std::map<Edge, int> boundaryEdgeCounts;
        bool foundCavityTriangle = false;
        for (std::size_t triangleIndex = 0; triangleIndex < triangleMesh.triangles.size(); ++triangleIndex) {
            if (!triangleMesh.triangles[triangleIndex].isPresent) continue;
            const Triangle& t = triangleMesh.triangles[triangleIndex].vertexIndices;
            const double inBallValue = InBall(points[t[0]], points[t[1]], points[t[2]], points[pointIndex]);
            const bool removeTriangle = inBallValue > 0.0;
            trace << "  in-circle test ";
            writeTriangle(triangleIndex, t);
            trace << " against p" << pointIndex << " result=" << inBallValue
                << " remove=" << (removeTriangle ? "true" : "false") << "\n";
            if (removeTriangle) {
                trianglesToRemove[triangleIndex] = true; foundCavityTriangle = true;
                for (int edgeIndex = 0; edgeIndex < 3; ++edgeIndex) ++boundaryEdgeCounts[MakeEdge(t[edgeIndex], t[(edgeIndex + 1) % 3])];
            }
        }
        trace << "  cavity_found=" << (foundCavityTriangle ? "true" : "false") << "\n";
        if (!foundCavityTriangle) {
            trace << "  no triangle selected; Bowyer-Watson skips this input point\n";
            writeLiveMesh("mesh after skipped input point");
            continue;
        }
        trace << "  selected triangles for removal:\n";
        for (std::size_t triangleIndex = 0; triangleIndex < trianglesToRemove.size(); ++triangleIndex) {
            if (!trianglesToRemove[triangleIndex]) continue;
            trace << "    ";
            writeTriangle(triangleIndex, triangleMesh.triangles[triangleIndex].vertexIndices);
            trace << "\n";
        }
        trace << "  cavity edge counts:\n";
        for (const auto& [edge, count] : boundaryEdgeCounts)
            trace << "    edge " << edge[0] << "=(" << points[edge[0]].x << "," << points[edge[0]].y << ") -- "
                << edge[1] << "=(" << points[edge[1]].x << "," << points[edge[1]].y << ") count=" << count
                << (count == 1 ? " boundary" : " internal") << "\n";
        for (std::size_t triangleIndex = 0; triangleIndex < trianglesToRemove.size(); ++triangleIndex)
            if (trianglesToRemove[triangleIndex]) RemoveTriangle(triangleMesh, triangleIndex);
        trace << "  new faces from cavity boundary:\n";
        for (const auto& [edge, count] : boundaryEdgeCounts) if (count == 1) {
            Triangle t = Ccw({edge[0], edge[1], pointIndex}, points);
            const double orientation = Orient(points[t[0]], points[t[1]], points[t[2]]);
            const std::size_t candidateTriangleIndex = triangleMesh.triangles.size();
            bool added = false;
            if (orientation != 0.0) added = AddTriangle(triangleMesh, t);
            trace << "    ";
            writeTriangle(candidateTriangleIndex, t);
            trace << " orient=" << orientation << " added=" << (added ? "true" : "false") << "\n";
        }
        writeLiveMesh("mesh after input insertion");
        if (!triangleMesh.hasValidTopology) {
            trace << "  stopping insertion loop because mesh topology became invalid\n";
            break;
        }
    }
    std::vector<Triangle> triangles;
    trace << "\nreal mesh after all input insertions:\n";
    for (std::size_t triangleIndex = 0; triangleIndex < triangleMesh.triangles.size(); ++triangleIndex) {
        const StoredTriangle& triangle = triangleMesh.triangles[triangleIndex];
        if (!triangle.isPresent) continue;
        trace << "  ";
        writeTriangle(triangleIndex, triangle.vertexIndices);
        trace << "\n";
        triangles.push_back(triangle.vertexIndices);
    }
    trace << "\nraw triangles returned by Bowyer-Watson (count=" << triangles.size() << "):\n";
    for (std::size_t triangleIndex = 0; triangleIndex < triangles.size(); ++triangleIndex) {
        trace << "  ";
        writeTriangle(triangleIndex, triangles[triangleIndex]);
        trace << "\n";
    }
    trace << "Bowyer-Watson return topology=" << (triangleMesh.hasValidTopology ? "valid" : "invalid") << "\n";
    diagnosticTrace = trace.str();
    return triangleMesh.hasValidTopology ? triangles : std::vector<Triangle>{};
}

// Add a required loop edge by repeatedly flipping mesh edges that cross it.
// Stop when the edge exists or no allowed flip can make progress.
bool RecoverConstraint(const Edge& constraint, TriangleMesh& triangleMesh,
    const std::vector<Vector2d>& points, std::set<Edge>& fixedConstraintEdges, std::string& failureReason) {
    std::vector<std::size_t> splitPoints{constraint[0], constraint[1]};
    for (std::size_t i = 0; i < points.size(); ++i)
        if (i != constraint[0] && i != constraint[1] && OnSegment(points[constraint[0]], points[constraint[1]], points[i]))
            splitPoints.push_back(i);
    const Vector2d direction = points[constraint[1]] - points[constraint[0]];
    const bool sortX = std::abs(direction.x) >= std::abs(direction.y);
    std::sort(splitPoints.begin(), splitPoints.end(), [&](std::size_t a, std::size_t b) {
        return sortX ? (direction.x >= 0.0 ? points[a].x < points[b].x : points[a].x > points[b].x)
                     : (direction.y >= 0.0 ? points[a].y < points[b].y : points[a].y > points[b].y);
    });
    splitPoints.erase(std::unique(splitPoints.begin(), splitPoints.end()), splitPoints.end());
    for (std::size_t i = 1; i < splitPoints.size(); ++i) {
        const Edge segment = MakeEdge(splitPoints[i - 1], splitPoints[i]);
        if (!RecoverConstraintSingle(segment, triangleMesh, points, fixedConstraintEdges, failureReason)) {
            failureReason = "subsegment [" + std::to_string(segment[0]) + ", " + std::to_string(segment[1]) +
                "]: " + failureReason;
            return false;
        }
    }
    return true;
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
void BuildConstraints(const InputGeometry& input, std::set<Edge>& constraints) {
    // Record each contour edge, including the edge from its last vertex back to its first.
    for (const ContourIndices& contour : input.contours) for (std::size_t i = 0; i < contour.vertices.size(); ++i) {
        const std::size_t startVertexIndex = contour.vertices[i], endVertexIndex = contour.vertices[(i + 1) % contour.vertices.size()];
        constraints.insert(MakeEdge(startVertexIndex, endVertexIndex));
    }
}

// Create the initial triangle mesh, recover all required contour edges, then legalize its free edges.
// Stop at the first failed stage and return its message to the caller.
bool BuildConstrainedTriangulation(const InputGeometry& input, const std::set<Edge>& constraints,
    std::vector<Triangle>& triangles, std::string& error) {
    // Build the initial triangulation, recover constraints, and legalize unconstrained edges in order.
    std::vector<Vector2d> points;
    std::string bowyerWatsonTrace;
    std::vector<Triangle> initialTriangles = BowyerWatson(input.points, points, bowyerWatsonTrace);
    if (initialTriangles.empty()) {
        error = "Could not triangulate input vertices.\n\n" + bowyerWatsonTrace;
        return false;
    }
    TriangleMesh triangleMesh;
    for (const Triangle& triangle : initialTriangles) {
        if (!AddTriangle(triangleMesh, triangle)) {
            error = "Bowyer-Watson faces could not form valid adjacency.\n\n" + bowyerWatsonTrace;
            return false;
        }
    }
    std::map<Edge, std::pair<bool, std::size_t>> initialConstraintStatus;
    for (const Edge& edge : constraints)
        initialConstraintStatus.emplace(edge, std::pair{HasEdge(triangleMesh, edge),
            CountProperCrossingEdges(edge, triangleMesh, points)});

    std::set<Edge> fixedConstraintEdges;
    for (const Edge& edge : constraints) {
        std::string failureReason;
        if (!RecoverConstraint(edge, triangleMesh, points, fixedConstraintEdges, failureReason)) {
            const auto [initiallyPresent, initialCrossings] = initialConstraintStatus.at(edge);
            error = "Could not recover contour constraint [" + std::to_string(edge[0]) + ", " +
                std::to_string(edge[1]) + "]: " + failureReason + " (initial seed: raw_triangle_edge_present=" +
                (TrianglesContainEdge(edge, initialTriangles) ? "true" : "false") + ", edge_map_present=" +
                (initiallyPresent ? "true" : "false") + ", raw_endpoint_triangles=" +
                DescribeRawEndpointTriangles(edge, initialTriangles, points) + ", proper_crossings=" +
                std::to_string(initialCrossings) + ", current_endpoint_incident_edges=" +
                std::to_string(CountIncidentEdges(edge[0], triangleMesh)) + "/" +
                std::to_string(CountIncidentEdges(edge[1], triangleMesh)) + "), local mesh vertices: " +
                DescribeConstraintNeighborhood(edge, triangleMesh, points) +
                "\n\n" + bowyerWatsonTrace;
            return false;
        }
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
        mesh.triangles.push_back({triangle, {}, TriangleRegion::Traversable});
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

// Append one region's vertices, faces, and constraints while shifting its local indices.
void AppendRegionMesh(const InputGeometry& input, const std::set<Edge>& constraints,
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

// Triangulate only pre-sanitized walkable regions. For each region, preserve any surviving source
// IDs, recover its contour constraints, filter faces by outer/hole membership, and require vertex
// coverage. Adjacency is validated on the combined draft before it can become an immutable NavMesh.
GenerationResult ConstrainedDelaunay::Triangulate(const detail::WalkableRegionSet& regions) const {
    std::string error;
    ConstrainedMesh combined;
    std::size_t regionCount = 0;
    for (const detail::WalkableRegion& region : regions.Regions()) {
        InputGeometry input;
        // Offset vertices have no source identity unless the builder preserved an exact source match.
        auto addContour = [&](const std::vector<Vector2d>& contour,
            const std::vector<std::optional<std::uint64_t>>& sourceIds) {
            // Each region uses local indices shared by its point array, output vertices, and contours.
            ContourIndices contourIndices;
            contourIndices.vertices.reserve(contour.size());
            for (std::size_t i = 0; i < contour.size(); ++i) {
                const Vector2d& position = contour[i];
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
        std::vector<Triangle> triangles;
        // Delaunay seed, constraint recovery, then unconstrained-edge legalization form one failure-aware stage.
        if (!BuildConstrainedTriangulation(input, constraints, triangles, error))
            return {GenerationStatus::Failed, {}, "Could not triangulate offset contours: " + error};
        KeepTrianglesInsideRegion(input, triangles, input.mesh);
        if (!UsesEveryInputVertex(input.mesh))
            return {GenerationStatus::Failed, {}, "An offset walkable component was not completely triangulated."};

        AppendRegionMesh(input, constraints, combined);
        ++regionCount;
    }

    if (regionCount == 0 || combined.triangles.empty())
        return {GenerationStatus::Failed, {}, "Agent radius leaves no triangulatable walkable area."};
    // This final pass checks reciprocal links and manifold edge use; only then is the draft published.
    if (!BuildFinalAdjacency(combined))
        return {GenerationStatus::Failed, {}, "Generated navmesh has non-manifold or invalid adjacency."};
    return {GenerationStatus::Success, NavMesh(std::move(combined)), "Navmesh baked successfully."};
}

} // namespace cdt
