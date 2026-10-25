#include "validation/GeometryValidator.hpp"
#include "core/GeometryPredicates.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace cdt {
namespace {

struct Tolerance {
    double length{};
    double area{};
};

// Fast ID-to-vertex lookup shared by one validation pass, avoiding repeated scans of all vertices.
using VerticesById = std::unordered_map<VertexId, const Vertex*>;

// Find a vertex in the lookup table, returning null when an edge or loop refers to a missing ID.
const Vertex* LookupVertex(const VerticesById& vertices_by_id, VertexId id) {
    const auto vertex_iterator = vertices_by_id.find(id);
    return vertex_iterator == vertices_by_id.end() ? nullptr : vertex_iterator->second;
}

// --- Scale-aware tolerance helpers ---

// Build length and area tolerances proportional to the geometry's characteristic scale.
Tolerance ForScale(double scale) {
    const double normalized_scale = std::max(scale, std::numeric_limits<double>::min());
    // Keep geometric comparisons relative to the loop's size, including its area tests.
    const double length = 64.0 * std::numeric_limits<double>::epsilon() * normalized_scale;
    return {length, length * normalized_scale};
}

// Measure a loop's bounding box diagonal to decide how much tiny calculation error to allow.
// Missing IDs are skipped here; the later validity check rejects loops that contain them.
double LoopScale(const VerticesById& vertices_by_id, const GeometryLoop& loop) {
    if (loop.vertices.empty()) return 0.0;
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const VertexId id : loop.vertices) {
        const Vertex* vertex = LookupVertex(vertices_by_id, id);
        if (!vertex) continue;
        min_x = std::min(min_x, vertex->position.x);
        min_y = std::min(min_y, vertex->position.y);
        max_x = std::max(max_x, vertex->position.x);
        max_y = std::max(max_y, vertex->position.y);
    }
    return std::hypot(max_x - min_x, max_y - min_y);
}

// Compute the signed 2D cross product of vectors AB and AC.
double Cross(Vector2d a, Vector2d b, Vector2d c) {
    return geometry::Cross2D(b - a, c - a);
}

// --- Segment and loop geometry predicates ---

// Check whether a point lies on a segment within the supplied scale-aware tolerance.
bool OnSegment(Vector2d a, Vector2d b, Vector2d p, const Tolerance& tolerance) {
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const double cross_tolerance = tolerance.length * std::max(length, tolerance.length);
    return std::abs(Cross(a, b, p)) <= cross_tolerance &&
        p.x >= std::min(a.x, b.x) - tolerance.length && p.x <= std::max(a.x, b.x) + tolerance.length &&
        p.y >= std::min(a.y, b.y) - tolerance.length && p.y <= std::max(a.y, b.y) + tolerance.length;
}

// Detect proper crossings, endpoint touches, and collinear overlap between two segments.
bool SegmentsIntersect(Vector2d a, Vector2d b, Vector2d c, Vector2d d, const Tolerance& tolerance) {
    const double ab_c = Cross(a, b, c), ab_d = Cross(a, b, d);
    const double cd_a = Cross(c, d, a), cd_b = Cross(c, d, b);
    const double ab_tolerance = tolerance.length * std::max(std::hypot(b.x - a.x, b.y - a.y), tolerance.length);
    const double cd_tolerance = tolerance.length * std::max(std::hypot(d.x - c.x, d.y - c.y), tolerance.length);
    if (((ab_c > ab_tolerance && ab_d < -ab_tolerance) || (ab_c < -ab_tolerance && ab_d > ab_tolerance)) &&
        ((cd_a > cd_tolerance && cd_b < -cd_tolerance) || (cd_a < -cd_tolerance && cd_b > cd_tolerance))) return true;
    return OnSegment(a, b, c, tolerance) || OnSegment(a, b, d, tolerance) ||
        OnSegment(c, d, a, tolerance) || OnSegment(c, d, b, tolerance);
}

// Compare every edge in the first loop with every edge in the second and stop at the first
// crossing, touch, or overlap. Missing endpoints are skipped, as in the previous validator.
bool LoopsIntersect(const VerticesById& vertices_by_id, const GeometryLoop& a, const GeometryLoop& b,
    const Tolerance& tolerance) {
    for (std::size_t i = 0; i < a.vertices.size(); ++i) {
        const Vertex* a0 = LookupVertex(vertices_by_id, a.vertices[i]);
        const Vertex* a1 = LookupVertex(vertices_by_id, a.vertices[(i + 1) % a.vertices.size()]);
        if (!a0 || !a1) continue;
        for (std::size_t j = 0; j < b.vertices.size(); ++j) {
            const Vertex* b0 = LookupVertex(vertices_by_id, b.vertices[j]);
            const Vertex* b1 = LookupVertex(vertices_by_id, b.vertices[(j + 1) % b.vertices.size()]);
            if (b0 && b1 && SegmentsIntersect(a0->position, a1->position,
                b0->position, b1->position, tolerance)) return true;
        }
    }
    return false;
}

// Check that every loop vertex exists and is finite, every edge has usable length, the area is
// non-zero, turns do not double back, and non-adjacent edges do not cross.
bool IsIndividuallyValid(const VerticesById& vertices_by_id, const GeometryLoop& loop, const Tolerance& tolerance) {
    if (loop.vertices.size() < 3) return false;
    const Vertex* origin_vertex = LookupVertex(vertices_by_id, loop.vertices.front());
    if (!origin_vertex) return false;
    std::vector<Vector2d> polygon;
    polygon.reserve(loop.vertices.size());
    for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
        const Vertex* a = LookupVertex(vertices_by_id, loop.vertices[i]);
        const Vertex* b = LookupVertex(vertices_by_id, loop.vertices[(i + 1) % loop.vertices.size()]);
        if (!a || !b) return false;
        if (!std::isfinite(a->position.x) || !std::isfinite(a->position.y) ||
            !std::isfinite(b->position.x) || !std::isfinite(b->position.y)) return false;
        if (std::hypot(b->position.x - a->position.x, b->position.y - a->position.y) <= tolerance.length)
            return false;
        polygon.push_back(a->position);
    }
    // Reject a collapsed loop before checking its turn directions or crossing edges.
    const double twice_area = geometry::SignedTwiceArea(polygon);
    if (std::abs(twice_area) <= tolerance.area) return false;

    // A valid loop may have straight turns, but it must not reverse back over the previous edge.
    for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
        const Vertex* previous = LookupVertex(vertices_by_id, loop.vertices[(i + loop.vertices.size() - 1) % loop.vertices.size()]);
        const Vertex* current = LookupVertex(vertices_by_id, loop.vertices[i]);
        const Vertex* next = LookupVertex(vertices_by_id, loop.vertices[(i + 1) % loop.vertices.size()]);
        if (!previous || !current || !next) return false;
        const Vector2d incoming = current->position - previous->position;
        const Vector2d outgoing = next->position - current->position;
        const double incoming_length = std::hypot(incoming.x, incoming.y);
        const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
        const double cross_tolerance = tolerance.length * std::max(incoming_length, outgoing_length);
        // Collinear consecutive edges are valid only when they continue in the same direction.
        if (std::abs(incoming.x * outgoing.y - incoming.y * outgoing.x) <= cross_tolerance &&
            incoming.dot(outgoing) < 0.0) return false;
    }

    // Adjacent edges share an endpoint by construction, so only non-adjacent pairs can self-cross.
    // Compare only non-adjacent edges because neighboring edges share an endpoint by design.
    for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
        const Vertex* a = LookupVertex(vertices_by_id, loop.vertices[i]);
        const Vertex* b = LookupVertex(vertices_by_id, loop.vertices[(i + 1) % loop.vertices.size()]);
        for (std::size_t j = i + 1; j < loop.vertices.size(); ++j) {
            if (j == (i + 1) % loop.vertices.size() || i == (j + 1) % loop.vertices.size()) continue;
            const Vertex* c = LookupVertex(vertices_by_id, loop.vertices[j]);
            const Vertex* d = LookupVertex(vertices_by_id, loop.vertices[(j + 1) % loop.vertices.size()]);
            if (a && b && c && d && SegmentsIntersect(a->position, b->position,
                c->position, d->position, tolerance))
                return false;
        }
    }
    return true;
}

// Test whether a point lies strictly inside a loop: boundary points count as outside, then the
// existing point-in-polygon rule handles the remaining case.
bool ContainsPoint(const VerticesById& vertices_by_id, const GeometryLoop& polygon, Vector2d point,
    const Tolerance& tolerance) {
    std::vector<Vector2d> points;
    points.reserve(polygon.vertices.size());
    for (std::size_t i = 0, j = polygon.vertices.size() - 1; i < polygon.vertices.size(); j = i++) {
        const Vertex* a = LookupVertex(vertices_by_id, polygon.vertices[i]);
        const Vertex* b = LookupVertex(vertices_by_id, polygon.vertices[j]);
        if (!a || !b) return false;
        // Boundary contact is deliberately not containment; callers require strict interior points.
        if (OnSegment(a->position, b->position, point, tolerance)) return false;
        points.push_back(a->position);
    }
    return geometry::PointInPolygon(points, point, geometry::BoundaryPolicy::Outside);
}

// A candidate is inside only when its boundary does not touch the border and every candidate
// vertex passes the strict point-in-polygon check.
bool EntirelyInside(const VerticesById& vertices_by_id, const GeometryLoop& candidate, const GeometryLoop& border,
    const Tolerance& tolerance) {
    if (LoopsIntersect(vertices_by_id, candidate, border, tolerance)) return false;
    // Checking every candidate vertex rules out a loop that crosses out and back through the border.
    return std::all_of(candidate.vertices.begin(), candidate.vertices.end(), [&](VertexId id) {
        const Vertex* vertex = LookupVertex(vertices_by_id, id);
        return vertex && ContainsPoint(vertices_by_id, border, vertex->position, tolerance);
    });
}

// --- Individual-loop validation ---

// Create one report entry for each model loop, preserving model order, and check each loop on its
// own using the scale already measured once for this validation pass.
void InitializeLoopResults(const GeometryModel& model, const VerticesById& vertices_by_id,
    const std::vector<double>& loop_scales, GeometryValidationReport& result) {
    result.loops.resize(model.Loops().size());
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        result.loops[i].loop_index = i;
        result.loops[i].individually_valid = IsIndividuallyValid(vertices_by_id, model.Loops()[i],
            ForScale(loop_scales[i]));
    }
}

// Choose the earliest completed individually valid border and mark it active and eligible.
void SelectActiveBorder(const GeometryModel& model, GeometryValidationReport& result) {
    std::vector<std::size_t> border_candidates;
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        const GeometryLoop& loop = model.Loops()[i];
        if (loop.type == EdgeType::Border && result.loops[i].individually_valid)
            border_candidates.push_back(i);
    }

    // Completion order preserves the first valid border chosen by the existing model behavior.
    std::sort(border_candidates.begin(), border_candidates.end(), [&](std::size_t a, std::size_t b) {
        return model.Loops()[a].completion_order < model.Loops()[b].completion_order;
    });
    if (border_candidates.empty()) return;

    result.active_border_index = border_candidates.front();
    result.loops[*result.active_border_index].active_border = true;
    result.loops[*result.active_border_index].eligible = true;
}

// --- Whole-model obstacle validation ---

// For each individually valid obstacle, record whether it is wholly inside the chosen border and
// whether its edges touch or cross that border. This establishes the set used by pair checks.
void ClassifyObstaclesAgainstBorder(const GeometryModel& model, const VerticesById& vertices_by_id,
    const std::vector<double>& loop_scales, GeometryValidationReport& result) {
    // Without a valid border, there is no inside/outside region for obstacles to join.
    if (!result.active_border_index) return;
    const GeometryLoop& border = model.Loops()[*result.active_border_index];

    // Establish containment for all obstacles first, so conflict checks use a stable candidate set.
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        const GeometryLoop& loop = model.Loops()[i];
        // Invalid loops cannot be accepted as bake obstacles, so skip their containment work.
        if (loop.type != EdgeType::Obstacle || !result.loops[i].individually_valid) continue;

        const Tolerance tolerance = ForScale(std::max(loop_scales[i], loop_scales[*result.active_border_index]));
        result.loops[i].inside_active_border = EntirelyInside(vertices_by_id, loop, border, tolerance);
        result.loops[i].intersects_conflict = LoopsIntersect(vertices_by_id, loop, border, tolerance);
    }
}

// Compare each pair of valid, contained obstacles once. When a pair touches or crosses, mark both
// as conflicting; after all pairs are checked, allow only contained obstacles with no conflict.
void DetectObstacleConflicts(const GeometryModel& model, const VerticesById& vertices_by_id,
    const std::vector<double>& loop_scales, GeometryValidationReport& result) {
    // Start the second index after the first so each unordered obstacle pair is tested only once.
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        const GeometryLoop& loop = model.Loops()[i];
        // Only valid obstacles inside the border can conflict with accepted obstacles.
        if (loop.type != EdgeType::Obstacle || !result.loops[i].individually_valid ||
            !result.loops[i].inside_active_border) continue;

        for (std::size_t j = i + 1; j < model.Loops().size(); ++j) {
            // Ignore borders, invalid loops, and obstacles outside the active border.
            if (model.Loops()[j].type != EdgeType::Obstacle ||
                !result.loops[j].individually_valid || !result.loops[j].inside_active_border) continue;

            const Tolerance tolerance = ForScale(std::max(loop_scales[i], loop_scales[j]));
            if (LoopsIntersect(vertices_by_id, loop, model.Loops()[j], tolerance)) {
                result.loops[i].intersects_conflict = true;
                result.loops[j].intersects_conflict = true;
            }
        }
    }
    // Once all pairs have been considered, mark contained obstacles eligible only when conflict-free.
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        const GeometryLoop& loop = model.Loops()[i];
        if (loop.type == EdgeType::Obstacle && result.loops[i].individually_valid &&
            result.loops[i].inside_active_border) {
            result.loops[i].eligible = !result.loops[i].intersects_conflict;
        }
    }
}

} // namespace

// Build the report in stages: prepare fast vertex access and loop sizes, check each loop, choose
// the active border, classify obstacles, then copy the usable loops into a bake-ready input.
GeometryValidationReport GeometryValidator::Validate(const GeometryModel& model) const {
    // Start with an empty report that will receive one result for each loop.
    GeometryValidationReport result;
    // Keep direct references to model vertices so repeated ID lookups do not scan the full list.
    VerticesById vertices_by_id;
    vertices_by_id.reserve(model.Vertices().size());
    for (const Vertex& vertex : model.Vertices()) vertices_by_id.emplace(vertex.id, &vertex);
    // Keep each loop's measured size because later border and obstacle checks use it repeatedly.
    std::vector<double> loop_scales;
    loop_scales.reserve(model.Loops().size());
    for (const GeometryLoop& loop : model.Loops()) loop_scales.push_back(LoopScale(vertices_by_id, loop));

    InitializeLoopResults(model, vertices_by_id, loop_scales, result);
    SelectActiveBorder(model, result);
    ClassifyObstaclesAgainstBorder(model, vertices_by_id, loop_scales, result);
    DetectObstacleConflicts(model, vertices_by_id, loop_scales, result);
    if (result.active_border_index) {
        // Copy one accepted model loop so later bake work can use its coordinates without reading
        // the editable model again. The earlier validity checks guarantee that its IDs resolve.
        const auto copy_validated_loop = [&](std::size_t index) {
            const GeometryLoop& source = model.Loops()[index];
            ValidatedLoop loop;
            loop.source_loop_index = index;
            loop.vertex_ids = source.vertices;
            loop.vertices.reserve(source.vertices.size());
            for (VertexId id : source.vertices)
                loop.vertices.push_back(LookupVertex(vertices_by_id, id)->position);
            return loop;
        };
        // Store the chosen border and each eligible obstacle in the bake-ready report.
        ValidatedLoop border = copy_validated_loop(*result.active_border_index);
        std::vector<ValidatedLoop> obstacles;
        // Keep obstacle order the same as the model so the generated signature stays stable.
        for (std::size_t i = 0; i < model.Loops().size(); ++i)
            if (i != *result.active_border_index && result.loops[i].eligible &&
                model.Loops()[i].type == EdgeType::Obstacle) obstacles.push_back(copy_validated_loop(i));
        result.validated_navmesh_input = ValidatedNavmeshInput(std::move(border), std::move(obstacles));
    }
    return result;
}

} // namespace cdt
