#include "cdt/ConstrainedDelaunay.hpp"
#include "core/GeometryPredicates.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <optional>
#include <set>
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
    std::vector<Triangle>& result) {
    if (polygon.size() < 3) return false;
    const double area2 = PolygonArea2(polygon, points);
    if (area2 == 0.0) return false;
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
            return false;
        }
    }
    if (Orient(points[polygon[0]], points[polygon[1]], points[polygon[2]]) == 0.0) return false;
    result.push_back(Ccw({polygon[0], polygon[1], polygon[2]}, points));
    return true;
}

bool RecoverByCavity(const Edge& constraint, TriangleMesh& triangleMesh, const std::vector<Vector2d>& points,
    const std::set<Edge>& fixedConstraintEdges) {
    std::vector<Edge> crossedEdges;
    for (const auto& [edge, uses] : triangleMesh.trianglesByEdge) {
        (void)uses;
        if (ProperlyIntersects(points[constraint[0]], points[constraint[1]], points[edge[0]], points[edge[1]])) {
            if (fixedConstraintEdges.contains(edge)) return false;
            crossedEdges.push_back(edge);
        }
    }
    if (crossedEdges.empty()) return false;
    std::set<TriangleIndex> removed;
    for (const Edge& edge : crossedEdges) {
        const auto it = triangleMesh.trianglesByEdge.find(edge);
        if (it == triangleMesh.trianglesByEdge.end() || it->second.triangleCount != 2) return false;
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
    if (graph[constraint[0]].size() != 2 || graph[constraint[1]].size() != 2) return false;

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
    if (first.empty()) return false;
    auto second = makePath(graph[constraint[0]][1]);
    if (second.empty() || first == second) return false;

    std::vector<Triangle> fill;
    if (!TriangulateSimplePolygon(std::move(first), points, fill)) return false;
    if (!TriangulateSimplePolygon(std::move(second), points, fill)) return false;
    TriangleMesh candidate = triangleMesh;
    for (TriangleIndex index : removed) RemoveTriangle(candidate, index);
    if (!candidate.hasValidTopology) return false;
    for (const Triangle& t : fill) {
        if (Orient(points[t[0]], points[t[1]], points[t[2]]) == 0.0) return false;
        if (!AddTriangle(candidate, t)) return false;
    }
    if (!candidate.hasValidTopology || !HasEdge(candidate, constraint)) return false;
    for (const Edge& fixed : fixedConstraintEdges) if (!HasEdge(candidate, fixed)) return false;
    triangleMesh = std::move(candidate);
    return true;
}

bool FlipEdge(TriangleMesh& triangleMesh, const Edge& edge, const std::vector<Vector2d>& points);

bool RecoverConstraintSingle(const Edge& constraint, TriangleMesh& triangleMesh,
    const std::vector<Vector2d>& points, std::set<Edge>& fixedConstraintEdges) {
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
            if (RecoverByCavity(constraint, triangleMesh, points, fixedConstraintEdges))
                return (fixedConstraintEdges.insert(constraint), true);
            return false;
        }
    }
    if (RecoverByCavity(constraint, triangleMesh, points, fixedConstraintEdges))
        return (fixedConstraintEdges.insert(constraint), true);
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

// Find the outside outline of all points, going counterclockwise around it.
// Points inside the outline are not returned. Points lying along a straight outside edge are kept.
bool BuildConvexHullBoundary(const std::vector<Vector2d>& points, std::vector<std::size_t>& hull) {
    // Start with an empty result so a failed call cannot leave old hull points behind.
    hull.clear();
    if (points.size() < 3) return false;

    // Sort point numbers, not the points themselves. This puts every point in a known
    // left-to-right order so the two scans below can trace the bottom and top outlines.
    std::vector<std::size_t> sorted(points.size());
    for (std::size_t i = 0; i < sorted.size(); ++i) sorted[i] = i;
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

bool Legalize(TriangleMesh& triangleMesh, const std::vector<Vector2d>& points,
    const std::set<Edge>& fixedConstraintEdges);

// Add interior points one at a time, replacing triangles whose circles contain each point.
// The real convex hull remains the mesh boundary throughout cavity removal.
std::vector<Triangle> BowyerWatson(const std::vector<Vector2d>& input, std::vector<Vector2d>& points) {
    points = input;
    if (points.size() < 3) return {};
    double minimumX = points[0].x, maximumX = minimumX, minimumY = points[0].y, maximumY = minimumY;
    for (const Vector2d& p : points) {
        minimumX = std::min(minimumX, p.x); maximumX = std::max(maximumX, p.x);
        minimumY = std::min(minimumY, p.y); maximumY = std::max(maximumY, p.y);
    }
    const double boundingBoxSpan = std::max(maximumX - minimumX, maximumY - minimumY);
    if (!std::isfinite(boundingBoxSpan) || boundingBoxSpan <= 0.0) return {};
    const std::size_t n = points.size();
    TriangleMesh triangleMesh;

    std::vector<std::size_t> hull;
    if (!BuildConvexHullBoundary(points, hull)) return {};

    std::vector<Triangle> hullTriangles;
    if (!TriangulateSimplePolygon(hull, points, hullTriangles)) return {};
    for (const Triangle& triangle : hullTriangles)
        if (!AddTriangle(triangleMesh, triangle)) return {};
    const std::set<Edge> noFixedConstraints;
    if (!Legalize(triangleMesh, points, noFixedConstraints)) return {};
    std::vector<bool> isHullVertex(n, false);
    for (std::size_t vertex : hull) isHullVertex[vertex] = true;

    // Point insertion: remove triangles whose circumcircles contain the point, then fill the cavity
    // from edges that occur only once on the removed triangles' boundary.
    for (std::size_t pointIndex = 0; pointIndex < n; ++pointIndex) {
        if (isHullVertex[pointIndex]) continue;
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
    std::vector<Triangle> triangles;
    for (const StoredTriangle& triangle : triangleMesh.triangles) {
        if (!triangle.isPresent) continue;
        triangles.push_back(triangle.vertexIndices);
    }
    return triangleMesh.hasValidTopology ? triangles : std::vector<Triangle>{};
}

// Add a required loop edge by repeatedly flipping mesh edges that cross it.
// Stop when the edge exists or no allowed flip can make progress.
bool RecoverConstraint(const Edge& constraint, TriangleMesh& triangleMesh,
    const std::vector<Vector2d>& points, std::set<Edge>& fixedConstraintEdges) {
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
        if (!RecoverConstraintSingle(segment, triangleMesh, points, fixedConstraintEdges)) return false;
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
    std::vector<Triangle> initialTriangles = BowyerWatson(input.points, points);
    if (initialTriangles.empty()) {
        error = "Could not triangulate input vertices.";
        return false;
    }
    TriangleMesh triangleMesh;
    for (const Triangle& triangle : initialTriangles) {
        if (!AddTriangle(triangleMesh, triangle)) {
            error = "Bowyer-Watson faces could not form valid adjacency.";
            return false;
        }
    }

    std::set<Edge> fixedConstraintEdges;
    for (const Edge& edge : constraints) {
        if (!RecoverConstraint(edge, triangleMesh, points, fixedConstraintEdges)) {
            error = "Could not recover every contour constraint.";
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
