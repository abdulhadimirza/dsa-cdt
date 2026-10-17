#include "validation/GeometryValidator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cdt {
namespace {

struct Tolerance {
    double length{};
    double area{};
};

// --- Scale-aware tolerance helpers ---

// Build length and area tolerances proportional to the geometry's characteristic scale.
Tolerance ForScale(double scale) {
    const double normalized_scale = std::max(scale, std::numeric_limits<double>::min());
    // Keep geometric comparisons relative to the loop's size, including its area tests.
    const double length = 64.0 * std::numeric_limits<double>::epsilon() * normalized_scale;
    return {length, length * normalized_scale};
}

// Return the diagonal length of a loop's bounding box, ignoring missing vertex references.
double LoopScale(const GeometryModel& model, const GeometryLoop& loop) {
    if (loop.vertices.empty()) return 0.0;
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const VertexId id : loop.vertices) {
        const Vertex* vertex = model.FindVertex(id);
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
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
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

// Return whether any edge in one loop intersects any edge in the other loop.
bool LoopsIntersect(const GeometryModel& model, const GeometryLoop& a, const GeometryLoop& b,
    const Tolerance& tolerance) {
    for (std::size_t i = 0; i < a.vertices.size(); ++i) {
        const Vertex* a0 = model.FindVertex(a.vertices[i]);
        const Vertex* a1 = model.FindVertex(a.vertices[(i + 1) % a.vertices.size()]);
        if (!a0 || !a1) continue;
        for (std::size_t j = 0; j < b.vertices.size(); ++j) {
            const Vertex* b0 = model.FindVertex(b.vertices[j]);
            const Vertex* b1 = model.FindVertex(b.vertices[(j + 1) % b.vertices.size()]);
            if (b0 && b1 && SegmentsIntersect(a0->position, a1->position,
                b0->position, b1->position, tolerance)) return true;
        }
    }
    return false;
}

// Validate a loop's vertices, non-degenerate area and edges, and absence of self-intersections.
bool IsIndividuallyValid(const GeometryModel& model, const GeometryLoop& loop, const Tolerance& tolerance) {
    if (loop.vertices.size() < 3) return false;
    const Vertex* origin_vertex = model.FindVertex(loop.vertices.front());
    if (!origin_vertex) return false;
    const Vector2d origin = origin_vertex->position;
    double twice_area = 0.0;
    for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
        const Vertex* a = model.FindVertex(loop.vertices[i]);
        const Vertex* b = model.FindVertex(loop.vertices[(i + 1) % loop.vertices.size()]);
        if (!a || !b) return false;
        if (std::hypot(b->position.x - a->position.x, b->position.y - a->position.y) <= tolerance.length)
            return false;
        twice_area += Cross(origin, a->position, b->position);
    }
    if (std::abs(twice_area) <= tolerance.area) return false;

    for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
        const Vertex* previous = model.FindVertex(loop.vertices[(i + loop.vertices.size() - 1) % loop.vertices.size()]);
        const Vertex* current = model.FindVertex(loop.vertices[i]);
        const Vertex* next = model.FindVertex(loop.vertices[(i + 1) % loop.vertices.size()]);
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
    for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
        const Vertex* a = model.FindVertex(loop.vertices[i]);
        const Vertex* b = model.FindVertex(loop.vertices[(i + 1) % loop.vertices.size()]);
        for (std::size_t j = i + 1; j < loop.vertices.size(); ++j) {
            if (j == (i + 1) % loop.vertices.size() || i == (j + 1) % loop.vertices.size()) continue;
            const Vertex* c = model.FindVertex(loop.vertices[j]);
            const Vertex* d = model.FindVertex(loop.vertices[(j + 1) % loop.vertices.size()]);
            if (a && b && c && d && SegmentsIntersect(a->position, b->position,
                c->position, d->position, tolerance))
                return false;
        }
    }
    return true;
}

// Use ray casting to test strict polygon containment; points on the boundary are outside.
bool ContainsPoint(const GeometryModel& model, const GeometryLoop& polygon, Vector2d point,
    const Tolerance& tolerance) {
    bool inside = false;
    for (std::size_t i = 0, j = polygon.vertices.size() - 1; i < polygon.vertices.size(); j = i++) {
        const Vertex* a = model.FindVertex(polygon.vertices[i]);
        const Vertex* b = model.FindVertex(polygon.vertices[j]);
        if (!a || !b) return false;
        // Boundary contact is deliberately not containment; callers require strict interior points.
        if (OnSegment(a->position, b->position, point, tolerance)) return false;
        const bool spans = (a->position.y > point.y) != (b->position.y > point.y);
        if (spans) {
            const double x = (b->position.x - a->position.x) * (point.y - a->position.y) /
                (b->position.y - a->position.y) + a->position.x;
            if (point.x < x) inside = !inside;
        }
    }
    return inside;
}

// Require both non-intersecting boundaries and every candidate vertex strictly inside the border.
bool EntirelyInside(const GeometryModel& model, const GeometryLoop& candidate, const GeometryLoop& border,
    const Tolerance& tolerance) {
    if (LoopsIntersect(model, candidate, border, tolerance)) return false;
    return std::all_of(candidate.vertices.begin(), candidate.vertices.end(), [&](VertexId id) {
        const Vertex* vertex = model.FindVertex(id);
        return vertex && ContainsPoint(model, border, vertex->position, tolerance);
    });
}

// --- Individual-loop validation ---

// Allocate one result per model loop and compute each loop's independent validity.
void InitializeLoopResults(const GeometryModel& model, GeometryValidation& result) {
    result.loops.resize(model.Loops().size());
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        result.loops[i].loop_index = i;
        result.loops[i].individually_valid = IsIndividuallyValid(model, model.Loops()[i],
            ForScale(LoopScale(model, model.Loops()[i])));
    }
}

// Choose the earliest completed individually valid border and mark it active and eligible.
void SelectActiveBorder(const GeometryModel& model, GeometryValidation& result) {
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

// Classify valid obstacles against the selected border before obstacle-pair checks.
void ClassifyObstaclesAgainstBorder(const GeometryModel& model, GeometryValidation& result) {
    if (!result.active_border_index) return;
    const GeometryLoop& border = model.Loops()[*result.active_border_index];

    // Establish containment for all obstacles first, so conflict checks use a stable candidate set.
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        const GeometryLoop& loop = model.Loops()[i];
        if (loop.type != EdgeType::Obstacle || !result.loops[i].individually_valid) continue;

        const Tolerance tolerance = ForScale(std::max(LoopScale(model, loop), LoopScale(model, border)));
        result.loops[i].inside_active_border = EntirelyInside(model, loop, border, tolerance);
        result.loops[i].intersects_conflict = LoopsIntersect(model, loop, border, tolerance);
    }
}

// Mark contained obstacles ineligible when they intersect the border or another contained obstacle.
void DetectObstacleConflicts(const GeometryModel& model, GeometryValidation& result) {
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        const GeometryLoop& loop = model.Loops()[i];
        if (loop.type != EdgeType::Obstacle || !result.loops[i].individually_valid ||
            !result.loops[i].inside_active_border) continue;

        bool conflict = result.loops[i].intersects_conflict;
        for (std::size_t j = 0; j < model.Loops().size(); ++j) {
            if (i == j || model.Loops()[j].type != EdgeType::Obstacle ||
                !result.loops[j].individually_valid || !result.loops[j].inside_active_border) continue;

            const Tolerance tolerance = ForScale(std::max(LoopScale(model, loop),
                LoopScale(model, model.Loops()[j])));
            if (LoopsIntersect(model, loop, model.Loops()[j], tolerance)) conflict = true;
        }
        result.loops[i].intersects_conflict = conflict;
        result.loops[i].eligible = !conflict;
    }
}

} // namespace

GeometryValidation GeometryValidator::Validate(const GeometryModel& model) const {
    GeometryValidation result;
    InitializeLoopResults(model, result);
    SelectActiveBorder(model, result);
    ClassifyObstaclesAgainstBorder(model, result);
    DetectObstacleConflicts(model, result);
    return result;
}

} // namespace cdt
