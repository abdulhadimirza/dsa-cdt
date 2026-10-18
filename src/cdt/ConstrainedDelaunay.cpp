#include "cdt/ConstrainedDelaunay.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <set>
#include <stddef.h>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

#include "robust-predicates/geompred.hpp"

namespace cdt {
namespace {

using Edge = std::array<std::size_t, 2>;
using Triangle = std::array<std::size_t, 3>;

// Set up the exact geometry checks once. The predicates need this initialization before use.
void InitializePredicates() {
    static const bool initialized = [] { mp_float::exactinit(); return true; }();
    (void)initialized;
}

// Store the smaller index first. This lets code compare an edge without depending on its direction.
Edge MakeEdge(std::size_t a, std::size_t b) { return a < b ? Edge{a, b} : Edge{b, a}; }

// Return the signed side of C relative to AB. Other functions use the sign to test turns and crossings.
double Orient(const Vector2d& a, const Vector2d& b, const Vector2d& c) {
    InitializePredicates();
    const double pa[]{a.x, a.y}, pb[]{b.x, b.y}, pc[]{c.x, c.y};
    return geompred::orient2d(pa, pb, pc);
}

// Return the signed circle test for D against ABC. The triangulation uses it to choose triangles to replace.
double InBall(const Vector2d& a, const Vector2d& b, const Vector2d& c, const Vector2d& d) {
    InitializePredicates();
    const double pa[]{a.x, a.y}, pb[]{b.x, b.y}, pc[]{c.x, c.y}, pd[]{d.x, d.y};
    return geompred::inball2d(pa, pb, pc, pd);
}

// Reverse two indices when needed so later geometry checks receive a counterclockwise triangle.
Triangle Ccw(Triangle t, const std::vector<Vector2d>& points) {
    if (Orient(points[t[0]], points[t[1]], points[t[2]]) < 0.0) std::swap(t[1], t[2]);
    return t;
}

// Store a loop using indices into the shared point list, with its border or obstacle role.
struct LoopIndices { std::vector<std::size_t> vertices; bool obstacle{}; };

// Keep the output vertices, their positions, and the selected loops together during generation.
struct InputGeometry {
    ConstrainedMesh mesh;
    std::vector<Vector2d> points;
    std::vector<LoopIndices> loops;
};

// Test the loop edges with a ray from the point. This lets generation keep only triangles inside the border.
bool PointInLoop(const Vector2d& point, const LoopIndices& loop, const std::vector<Vector2d>& points) {
    // Begin: count horizontal-ray crossings. Each crossing toggles inside/outside; this classifies triangle centers.
    // Count crossings of a horizontal ray; an odd count means the point is inside the loop.
    bool inside = false;
    for (std::size_t i = 0, j = loop.vertices.size() - 1; i < loop.vertices.size(); j = i++) {
        const Vector2d& a = points[loop.vertices[i]];
        const Vector2d& b = points[loop.vertices[j]];
        if ((a.y > point.y) != (b.y > point.y) &&
            point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    // End: the odd/even crossing result is the point's loop membership.
    return inside;
}

// Compare which side each segment's endpoints lie on. Opposite sides on both segments mean they cross inside.
bool ProperlyIntersects(const Vector2d& a, const Vector2d& b, const Vector2d& c, const Vector2d& d) {
    const double ab_c = Orient(a, b, c), ab_d = Orient(a, b, d);
    const double cd_a = Orient(c, d, a), cd_b = Orient(c, d, b);
    return ((ab_c > 0.0 && ab_d < 0.0) || (ab_c < 0.0 && ab_d > 0.0)) &&
        ((cd_a > 0.0 && cd_b < 0.0) || (cd_a < 0.0 && cd_b > 0.0));
}

// Search the triangle edges for this edge. Constraint recovery uses this to know when it is finished.
bool HasEdge(const std::vector<Triangle>& triangles, const Edge& edge) {
    for (const Triangle& t : triangles)
        for (int i = 0; i < 3; ++i)
            if (MakeEdge(t[i], t[(i + 1) % 3]) == edge) return true;
    return false;
}

// Find the two triangles sharing an edge and replace their shared edge with the other diagonal.
// Reject boundary edges, non-convex pairs, and zero-area results so the mesh stays valid.
bool FlipEdge(std::vector<Triangle>& triangles, const Edge& edge, const std::vector<Vector2d>& points) {
    // Begin: find both faces touching the edge and the opposite corner of each face.
    // A flip is possible only when the edge is shared by exactly two triangles.
    std::vector<std::pair<std::size_t, std::size_t>> adjacent;
    for (std::size_t ti = 0; ti < triangles.size(); ++ti) {
        const Triangle& t = triangles[ti];
        for (int i = 0; i < 3; ++i)
            if (MakeEdge(t[i], t[(i + 1) % 3]) == edge)
                adjacent.emplace_back(ti, t[(i + 2) % 3]);
    }
    if (adjacent.size() != 2) return false;
    // End: the candidate edge has two adjacent triangles to check.

    // Begin: verify the replacement diagonal makes two non-flat triangles, then replace the old faces.
    // This keeps the mesh connected and avoids introducing zero-area faces.
    const std::size_t a = edge[0], b = edge[1], c = adjacent[0].second, d = adjacent[1].second;
    if (c == d || !ProperlyIntersects(points[a], points[b], points[c], points[d])) return false;
    Triangle first = Ccw({c, d, a}, points), second = Ccw({d, c, b}, points);
    if (Orient(points[first[0]], points[first[1]], points[first[2]]) == 0.0 ||
        Orient(points[second[0]], points[second[1]], points[second[2]]) == 0.0) return false;
    const std::size_t i0 = adjacent[0].first, i1 = adjacent[1].first;
    triangles[std::min(i0, i1)] = first;
    triangles[std::max(i0, i1)] = second;
    // End: both faces now use the replacement diagonal.
    return true;
}

// Add points one at a time, replacing triangles whose circles contain each point with new triangles.
// A large temporary triangle starts the process; it is removed before returning the mesh.
std::vector<Triangle> BowyerWatson(const std::vector<Vector2d>& input, std::vector<Vector2d>& points) {
    // Find the input bounds and create a temporary triangle around them so every point starts inside a face.
    points = input;
    double min_x = points[0].x, max_x = min_x, min_y = points[0].y, max_y = min_y;
    for (const Vector2d& p : points) {
        min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    }
    const double span = std::max(max_x - min_x, max_y - min_y);
    if (!std::isfinite(span) || span <= 0.0) return {};
    const double cx = (min_x + max_x) * 0.5, cy = (min_y + max_y) * 0.5;
    const std::size_t n = points.size();
    points.emplace_back(cx - 32.0 * span, cy - 16.0 * span);
    points.emplace_back(cx + 32.0 * span, cy - 16.0 * span);
    points.emplace_back(cx, cy + 32.0 * span);
    std::vector<Triangle> triangles{Ccw({n, n + 1, n + 2}, points)};

    // Begin: insert each input point by removing affected triangles and filling their exposed outline.
    // This updates the mesh incrementally as each point is added.
    for (std::size_t pi = 0; pi < n; ++pi) {
        std::vector<bool> bad(triangles.size(), false);
        std::map<Edge, int> boundary;
        bool found = false;
        for (std::size_t ti = 0; ti < triangles.size(); ++ti) {
            const Triangle& t = triangles[ti];
            if (InBall(points[t[0]], points[t[1]], points[t[2]], points[pi]) > 0.0) {
                bad[ti] = true; found = true;
                for (int e = 0; e < 3; ++e) ++boundary[MakeEdge(t[e], t[(e + 1) % 3])];
            }
        }
        if (!found) continue;
        std::vector<Triangle> kept;
        for (std::size_t ti = 0; ti < triangles.size(); ++ti) if (!bad[ti]) kept.push_back(triangles[ti]);
        for (const auto& [edge, count] : boundary) if (count == 1) {
            Triangle t = Ccw({edge[0], edge[1], pi}, points);
            if (Orient(points[t[0]], points[t[1]], points[t[2]]) != 0.0) kept.push_back(t);
        }
        triangles.swap(kept);
    }
    // End: all input points have been added to the temporary triangulation.

    // Remove faces that use temporary vertices and restore the point list to only the input vertices.
    triangles.erase(std::remove_if(triangles.begin(), triangles.end(), [n](const Triangle& t) {
        return t[0] >= n || t[1] >= n || t[2] >= n;
    }), triangles.end());
    points.resize(n);
    return triangles;
}

// Add a required loop edge by repeatedly flipping mesh edges that cross it.
// Stop when the edge exists or no allowed flip can make progress.
bool RecoverConstraint(const Edge& constraint, std::vector<Triangle>& triangles,
    const std::vector<Vector2d>& points, std::set<Edge>& fixed) {
    const std::size_t limit = triangles.size() * triangles.size() + 32;
    // Begin: flip crossing mesh edges until the required loop edge appears or no flip can help.
    // Mark recovered edges fixed so later steps do not remove the border or obstacle outline.
    for (std::size_t iteration = 0; iteration < limit; ++iteration) {
        if (HasEdge(triangles, constraint)) { fixed.insert(constraint); return true; }
        std::vector<Edge> crossing;
        for (const Triangle& t : triangles) for (int i = 0; i < 3; ++i) {
            const Edge e = MakeEdge(t[i], t[(i + 1) % 3]);
            if (e != constraint && ProperlyIntersects(points[constraint[0]], points[constraint[1]],
                points[e[0]], points[e[1]])) crossing.push_back(e);
        }
        std::sort(crossing.begin(), crossing.end());
        crossing.erase(std::unique(crossing.begin(), crossing.end()), crossing.end());
        bool flipped = false;
        for (const Edge& e : crossing) if (!fixed.contains(e) && FlipEdge(triangles, e, points)) {
            flipped = true; break;
        }
        if (!flipped) return false;
    }
    // End: reaching the limit means recovery did not make enough progress.
    return false;
}

// Check shared, non-constraint edges and flip ones that violate the circle rule.
// Repeat until no edge needs changing or the safety limit is reached.
bool Legalize(std::vector<Triangle>& triangles, const std::vector<Vector2d>& points,
    const std::set<Edge>& fixed) {
    const std::size_t limit = triangles.size() * triangles.size() * 8 + 64;
    // Begin: rebuild edge adjacency and inspect each shared edge for a Delaunay violation.
    // Boundary constraints are skipped because their shape must remain fixed.
    for (std::size_t iteration = 0; iteration < limit; ++iteration) {
        std::map<Edge, std::vector<std::size_t>> adjacent;
        for (std::size_t ti = 0; ti < triangles.size(); ++ti)
            for (int e = 0; e < 3; ++e) adjacent[MakeEdge(triangles[ti][e], triangles[ti][(e + 1) % 3])].push_back(ti);
        bool changed = false;
        for (const auto& [edge, faces] : adjacent) {
            if (faces.size() != 2 || fixed.contains(edge)) continue;
            auto opposite = [&](const Triangle& t) {
                for (std::size_t v : t) if (v != edge[0] && v != edge[1]) return v;
                return edge[0];
            };
            const std::size_t c = opposite(triangles[faces[0]]), d = opposite(triangles[faces[1]]);
            if (c == d || !ProperlyIntersects(points[edge[0]], points[edge[1]], points[c], points[d])) continue;
            Triangle t = Ccw({edge[0], edge[1], c}, points);
            if (InBall(points[t[0]], points[t[1]], points[t[2]], points[d]) > 0.0 &&
                FlipEdge(triangles, edge, points)) { changed = true; break; }
        }
        if (!changed) return true;
    }
    // End: the safety limit was reached before the mesh settled.
    return false;
}

// Copy one selected loop into the shared input lists and preserve its source vertex IDs.
// The index map avoids adding a shared model vertex twice; false means the loop data is incomplete.
bool AddLoop(const GeometryModel& model, const GeometryLoop& loop, bool obstacle,
    InputGeometry& input, std::unordered_map<VertexId, std::size_t>& index_by_id) {
    // Begin: resolve each loop vertex and assign a shared index for the mesh and triangle routines.
    // Invalid or non-finite vertices stop collection because later geometry checks need valid coordinates.
    LoopIndices indices;
    indices.obstacle = obstacle;
    for (VertexId id : loop.vertices) {
        const Vertex* vertex = model.FindVertex(id);
        if (!vertex || !std::isfinite(vertex->position.x) || !std::isfinite(vertex->position.y)) return false;
        auto [it, inserted] = index_by_id.emplace(id, input.mesh.vertices.size());
        if (inserted) {
            input.mesh.vertices.push_back({id, vertex->position});
            input.points.push_back(vertex->position);
        }
        indices.vertices.push_back(it->second);
    }
    if (indices.vertices.size() < 3) return false;
    input.loops.push_back(std::move(indices));
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
    // Begin: add the active border first because later code uses it as the domain boundary.
    std::unordered_map<VertexId, std::size_t> index_by_id;
    if (!AddLoop(model, model.Loops()[border_index], false, input, index_by_id)) {
        error = "Active border input is inconsistent.";
        return false;
    }
    // End: the border is the first loop in the input list.

    // Include only validated obstacle loops; other loops must not affect the generated mesh.
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        if (i == border_index || i >= validation.loops.size() || !validation.loops[i].eligible ||
            model.Loops()[i].type != EdgeType::Obstacle) continue;
        if (!AddLoop(model, model.Loops()[i], true, input, index_by_id)) {
            error = "Obstacle input is inconsistent.";
            return false;
        }
    }
    return true;
}

// Make one required mesh edge for each consecutive pair of vertices in each selected loop.
// These edges keep the generated triangles aligned with the border and obstacle outlines.
bool BuildConstraints(const InputGeometry& input, std::set<Edge>& constraints, std::string& error) {
    // Constraint splitting remains disabled for now. Each original loop edge is used as-is.
    // Begin: connect each loop vertex to the next, including the closing edge.
    // These are the exact edges the triangulation must preserve as boundaries.
    for (const LoopIndices& loop : input.loops) for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
        const std::size_t start = loop.vertices[i], finish = loop.vertices[(i + 1) % loop.vertices.size()];
        if (start == finish) {
            error = "Coincident constraint endpoints.";
            return false;
        }
        constraints.insert(MakeEdge(start, finish));
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
    // End: no duplicate coordinate pair was found.
    return false;
}

// Create the initial triangle mesh, add all required loop edges, then improve its unconstrained edges.
// Stop at the first failed stage and return its message to the caller.
bool BuildConstrainedTriangulation(const InputGeometry& input, const std::set<Edge>& constraints,
    std::vector<Triangle>& triangles, std::string& error) {
    // Begin: build triangles from the points, then recover and protect each required loop edge.
    // These stages create the constrained mesh that respects the selected outlines.
    std::vector<Vector2d> points;
    triangles = BowyerWatson(input.points, points);
    if (triangles.empty()) {
        error = "Could not triangulate input vertices.";
        return false;
    }

    std::set<Edge> fixed;
    for (const Edge& edge : constraints) if (!RecoverConstraint(edge, triangles, points, fixed)) {
        error = "Could not recover every boundary constraint.";
        return false;
    }
    if (!Legalize(triangles, points, fixed)) {
        error = "Constrained edge legalization did not converge.";
        return false;
    }
    // End: the mesh contains all constraints and its remaining edges have been improved.
    return true;
}

// Keep triangles inside the border and label those inside an obstacle as blocked.
// Triangle centers provide a simple point to test against each selected loop.
void AddTrianglesInsideLoops(const InputGeometry& input, const std::vector<Triangle>& triangles,
    ConstrainedMesh& mesh) {
    // Begin: classify each triangle by its center, keeping only those inside the border.
    // Obstacle membership changes the region label so navigation can treat those triangles as blocked.
    for (const Triangle& triangle : triangles) {
        const Vector2d center = (input.points[triangle[0]] + input.points[triangle[1]] +
            input.points[triangle[2]]) / 3.0;
        if (!PointInLoop(center, input.loops.front(), input.points)) continue;
        TriangleRegion region = TriangleRegion::Traversable;
        for (std::size_t loop_index = 1; loop_index < input.loops.size(); ++loop_index)
            if (PointInLoop(center, input.loops[loop_index], input.points)) {
                region = TriangleRegion::Obstacle;
                break;
            }
        mesh.triangles.push_back({triangle, region});
    }
    // End: the output mesh now contains only in-domain triangles with region labels.
}

// Check that clipping did not leave any selected input vertex without a triangle.
bool UsesEveryInputVertex(const ConstrainedMesh& mesh) {
    // Mark each referenced vertex, then require a non-empty mesh with no unused vertices.
    // Begin: record vertex use from all retained triangles.
    std::vector<bool> used(mesh.vertices.size(), false);
    for (const MeshTriangle& triangle : mesh.triangles)
        for (std::size_t vertex : triangle.vertices) used[vertex] = true;
    // End: accept only a non-empty mesh that uses every selected input vertex.
    return !mesh.triangles.empty() && std::find(used.begin(), used.end(), false) == used.end();
}

} // namespace

// Build a mesh from the active border and eligible obstacle loops.
// Input loops become constraints, triangles are selected by location, and failures return a message.
GenerationResult ConstrainedDelaunay::Generate(const GeometryModel& model,
    const GeometryValidation& validation) const {
    if (!validation.active_border_index)
        return {GenerationStatus::NoActiveBorder, {}, "No valid active border."};

    // Begin: gather and check source geometry before asking the triangulator to process it.
    InputGeometry input;
    std::string error;
    if (!CollectInputLoops(model, validation, input, error))
        return {GenerationStatus::Failed, {}, std::move(error)};
    // Build the required loop edges and reject point sets the triangle builder cannot use.
    std::set<Edge> constraints;
    if (!BuildConstraints(input, constraints, error))
        return {GenerationStatus::Failed, {}, std::move(error)};
    if (input.points.size() < 3)
        return {GenerationStatus::Failed, {}, "Not enough distinct input vertices."};
    if (HasCoincidentPoints(input.points))
        return {GenerationStatus::Failed, {}, "Coincident input vertices are unsupported."};
    // End: selected points and loop constraints are ready for triangulation.

    // Begin: build the constrained triangulation, clip it to the loops, and verify the output.
    // Each stage can stop with a message when it cannot produce a complete mesh.
    std::vector<Triangle> triangles;
    if (!BuildConstrainedTriangulation(input, constraints, triangles, error))
        return {GenerationStatus::Failed, {}, std::move(error)};

    AddTrianglesInsideLoops(input, triangles, input.mesh);
    for (const Edge& edge : constraints) input.mesh.constraints.push_back(edge);
    if (!UsesEveryInputVertex(input.mesh))
        return {GenerationStatus::Failed, {}, "The complete input domain was not triangulated."};
    // End: the mesh is complete and ready to return.
    return {GenerationStatus::Success, std::move(input.mesh), "Navmesh baked successfully."};
}

} // namespace cdt
