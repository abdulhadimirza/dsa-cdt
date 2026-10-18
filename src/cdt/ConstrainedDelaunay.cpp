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

void InitializePredicates() {
    static const bool initialized = [] { mp_float::exactinit(); return true; }();
    (void)initialized;
}

Edge MakeEdge(std::size_t a, std::size_t b) { return a < b ? Edge{a, b} : Edge{b, a}; }

double Orient(const Vector2d& a, const Vector2d& b, const Vector2d& c) {
    InitializePredicates();
    const double pa[]{a.x, a.y}, pb[]{b.x, b.y}, pc[]{c.x, c.y};
    return geompred::orient2d(pa, pb, pc);
}

double InBall(const Vector2d& a, const Vector2d& b, const Vector2d& c, const Vector2d& d) {
    InitializePredicates();
    const double pa[]{a.x, a.y}, pb[]{b.x, b.y}, pc[]{c.x, c.y}, pd[]{d.x, d.y};
    return geompred::inball2d(pa, pb, pc, pd);
}

Triangle Ccw(Triangle t, const std::vector<Vector2d>& points) {
    if (Orient(points[t[0]], points[t[1]], points[t[2]]) < 0.0) std::swap(t[1], t[2]);
    return t;
}

struct LoopIndices { std::vector<std::size_t> vertices; bool obstacle{}; };

bool PointInLoop(const Vector2d& point, const LoopIndices& loop, const std::vector<Vector2d>& points) {
    bool inside = false;
    for (std::size_t i = 0, j = loop.vertices.size() - 1; i < loop.vertices.size(); j = i++) {
        const Vector2d& a = points[loop.vertices[i]];
        const Vector2d& b = points[loop.vertices[j]];
        if ((a.y > point.y) != (b.y > point.y) &&
            point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
}

bool ProperlyIntersects(const Vector2d& a, const Vector2d& b, const Vector2d& c, const Vector2d& d) {
    const double ab_c = Orient(a, b, c), ab_d = Orient(a, b, d);
    const double cd_a = Orient(c, d, a), cd_b = Orient(c, d, b);
    return ((ab_c > 0.0 && ab_d < 0.0) || (ab_c < 0.0 && ab_d > 0.0)) &&
        ((cd_a > 0.0 && cd_b < 0.0) || (cd_a < 0.0 && cd_b > 0.0));
}

bool HasEdge(const std::vector<Triangle>& triangles, const Edge& edge) {
    for (const Triangle& t : triangles)
        for (int i = 0; i < 3; ++i)
            if (MakeEdge(t[i], t[(i + 1) % 3]) == edge) return true;
    return false;
}

bool FlipEdge(std::vector<Triangle>& triangles, const Edge& edge, const std::vector<Vector2d>& points) {
    std::vector<std::pair<std::size_t, std::size_t>> adjacent;
    for (std::size_t ti = 0; ti < triangles.size(); ++ti) {
        const Triangle& t = triangles[ti];
        for (int i = 0; i < 3; ++i)
            if (MakeEdge(t[i], t[(i + 1) % 3]) == edge)
                adjacent.emplace_back(ti, t[(i + 2) % 3]);
    }
    if (adjacent.size() != 2) return false;
    const std::size_t a = edge[0], b = edge[1], c = adjacent[0].second, d = adjacent[1].second;
    if (c == d || !ProperlyIntersects(points[a], points[b], points[c], points[d])) return false;
    Triangle first = Ccw({c, d, a}, points), second = Ccw({d, c, b}, points);
    if (Orient(points[first[0]], points[first[1]], points[first[2]]) == 0.0 ||
        Orient(points[second[0]], points[second[1]], points[second[2]]) == 0.0) return false;
    const std::size_t i0 = adjacent[0].first, i1 = adjacent[1].first;
    triangles[std::min(i0, i1)] = first;
    triangles[std::max(i0, i1)] = second;
    return true;
}

std::vector<Triangle> BowyerWatson(const std::vector<Vector2d>& input, std::vector<Vector2d>& points) {
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
    triangles.erase(std::remove_if(triangles.begin(), triangles.end(), [n](const Triangle& t) {
        return t[0] >= n || t[1] >= n || t[2] >= n;
    }), triangles.end());
    points.resize(n);
    return triangles;
}

bool RecoverConstraint(const Edge& constraint, std::vector<Triangle>& triangles,
    const std::vector<Vector2d>& points, std::set<Edge>& fixed) {
    const std::size_t limit = triangles.size() * triangles.size() + 32;
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
    return false;
}

bool Legalize(std::vector<Triangle>& triangles, const std::vector<Vector2d>& points,
    const std::set<Edge>& fixed) {
    const std::size_t limit = triangles.size() * triangles.size() * 8 + 64;
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
    return false;
}

} // namespace

GenerationResult ConstrainedDelaunay::Generate(const GeometryModel& model,
    const GeometryValidation& validation) const {
    if (!validation.active_border_index)
        return {GenerationStatus::NoActiveBorder, {}, "No valid active border."};

    ConstrainedMesh mesh;
    std::vector<Vector2d> source_points;
    std::vector<LoopIndices> loops;
    std::unordered_map<VertexId, std::size_t> index_by_id;
    auto add_loop = [&](const GeometryLoop& loop, bool obstacle) -> bool {
        LoopIndices indices; indices.obstacle = obstacle;
        for (VertexId id : loop.vertices) {
            const Vertex* vertex = model.FindVertex(id);
            if (!vertex || !std::isfinite(vertex->position.x) || !std::isfinite(vertex->position.y)) return false;
            auto [it, inserted] = index_by_id.emplace(id, mesh.vertices.size());
            if (inserted) {
                mesh.vertices.push_back({id, vertex->position});
                source_points.push_back(vertex->position);
            }
            indices.vertices.push_back(it->second);
        }
        if (indices.vertices.size() < 3) return false;
        loops.push_back(std::move(indices));
        return true;
    };

    const std::size_t border_index = *validation.active_border_index;
    if (border_index >= model.Loops().size() || border_index >= validation.loops.size() ||
        !validation.loops[border_index].active_border || !add_loop(model.Loops()[border_index], false))
        return {GenerationStatus::Failed, {}, "Active border input is inconsistent."};
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        if (i == border_index || i >= validation.loops.size() || !validation.loops[i].eligible ||
            model.Loops()[i].type != EdgeType::Obstacle) continue;
        if (!add_loop(model.Loops()[i], true)) return {GenerationStatus::Failed, {}, "Obstacle input is inconsistent."};
    }

    std::set<Edge> constraints;
    // Temporarily disabled constraint splitting. Recover each loop edge directly below.
    // for (const LoopIndices& loop : loops) for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
    //     const std::size_t start = loop.vertices[i], finish = loop.vertices[(i + 1) % loop.vertices.size()];
    //     if (start == finish) return {GenerationStatus::Failed, {}, "Coincident constraint endpoints."};
    //     const Vector2d direction = source_points[finish] - source_points[start];
    //     const double length_sq = direction.length_sq();
    //     std::vector<std::pair<double, std::size_t>> split_points{{0.0, start}, {1.0, finish}};
    //     for (std::size_t candidate = 0; candidate < source_points.size(); ++candidate) {
    //         if (candidate == start || candidate == finish ||
    //             Orient(source_points[start], source_points[finish], source_points[candidate]) != 0.0) continue;
    //         const double t = (source_points[candidate] - source_points[start]).dot(direction) / length_sq;
    //         if (t > 0.0 && t < 1.0) split_points.emplace_back(t, candidate);
    //     }
    //     std::sort(split_points.begin(), split_points.end());
    //     for (std::size_t split = 1; split < split_points.size(); ++split)
    //         constraints.insert(MakeEdge(split_points[split - 1].second, split_points[split].second));
    // }
    // Add each original loop edge without splitting it at other input vertices.
    for (const LoopIndices& loop : loops) for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
        const std::size_t start = loop.vertices[i], finish = loop.vertices[(i + 1) % loop.vertices.size()];
        if (start == finish) return {GenerationStatus::Failed, {}, "Coincident constraint endpoints."};
        constraints.insert(MakeEdge(start, finish));
    }
    if (source_points.size() < 3) return {GenerationStatus::Failed, {}, "Not enough distinct input vertices."};
    for (std::size_t i = 0; i < source_points.size(); ++i) for (std::size_t j = i + 1; j < source_points.size(); ++j)
        if (source_points[i] == source_points[j]) return {GenerationStatus::Failed, {}, "Coincident input vertices are unsupported."};

    std::vector<Vector2d> points;
    std::vector<Triangle> triangles = BowyerWatson(source_points, points);
    if (triangles.empty()) return {GenerationStatus::Failed, {}, "Could not triangulate input vertices."};
    std::set<Edge> fixed;
    for (const Edge& edge : constraints) if (!RecoverConstraint(edge, triangles, points, fixed))
        return {GenerationStatus::Failed, {}, "Could not recover every boundary constraint."};
    if (!Legalize(triangles, points, fixed))
        return {GenerationStatus::Failed, {}, "Constrained edge legalization did not converge."};

    for (const Triangle& t : triangles) {
        const Vector2d center = (source_points[t[0]] + source_points[t[1]] + source_points[t[2]]) / 3.0;
        if (!PointInLoop(center, loops.front(), source_points)) continue;
        TriangleRegion region = TriangleRegion::Traversable;
        for (std::size_t li = 1; li < loops.size(); ++li)
            if (PointInLoop(center, loops[li], source_points)) { region = TriangleRegion::Obstacle; break; }
        mesh.triangles.push_back({t, region});
    }
    for (const Edge& edge : constraints) mesh.constraints.push_back(edge);
    std::vector<bool> used(mesh.vertices.size(), false);
    for (const MeshTriangle& t : mesh.triangles) for (std::size_t v : t.vertices) used[v] = true;
    if (std::find(used.begin(), used.end(), false) != used.end() || mesh.triangles.empty())
        return {GenerationStatus::Failed, {}, "The complete input domain was not triangulated."};
    return {GenerationStatus::Success, std::move(mesh), "Navmesh baked successfully."};
}

} // namespace cdt
