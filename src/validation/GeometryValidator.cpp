#include "validation/GeometryValidator.hpp"

#include <algorithm>
#include <cmath>

namespace cdt {
namespace {

constexpr double epsilon = 1e-9;

double Cross(Vector2d a, Vector2d b, Vector2d c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool OnSegment(Vector2d a, Vector2d b, Vector2d p) {
    return std::abs(Cross(a, b, p)) <= epsilon &&
        p.x >= std::min(a.x, b.x) - epsilon && p.x <= std::max(a.x, b.x) + epsilon &&
        p.y >= std::min(a.y, b.y) - epsilon && p.y <= std::max(a.y, b.y) + epsilon;
}

bool SegmentsIntersect(Vector2d a, Vector2d b, Vector2d c, Vector2d d) {
    const double ab_c = Cross(a, b, c), ab_d = Cross(a, b, d);
    const double cd_a = Cross(c, d, a), cd_b = Cross(c, d, b);
    if (((ab_c > epsilon && ab_d < -epsilon) || (ab_c < -epsilon && ab_d > epsilon)) &&
        ((cd_a > epsilon && cd_b < -epsilon) || (cd_a < -epsilon && cd_b > epsilon))) return true;
    return OnSegment(a, b, c) || OnSegment(a, b, d) || OnSegment(c, d, a) || OnSegment(c, d, b);
}

bool LoopsIntersect(const GeometryModel& model, const GeometryLoop& a, const GeometryLoop& b) {
    for (std::size_t i = 0; i < a.vertices.size(); ++i) {
        const Vertex* a0 = model.FindVertex(a.vertices[i]);
        const Vertex* a1 = model.FindVertex(a.vertices[(i + 1) % a.vertices.size()]);
        if (!a0 || !a1) continue;
        for (std::size_t j = 0; j < b.vertices.size(); ++j) {
            const Vertex* b0 = model.FindVertex(b.vertices[j]);
            const Vertex* b1 = model.FindVertex(b.vertices[(j + 1) % b.vertices.size()]);
            if (b0 && b1 && SegmentsIntersect(a0->position, a1->position, b0->position, b1->position)) return true;
        }
    }
    return false;
}

bool IsIndividuallyValid(const GeometryModel& model, const GeometryLoop& loop) {
    if (loop.vertices.size() < 3) return false;
    double twice_area = 0.0;
    for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
        const Vertex* a = model.FindVertex(loop.vertices[i]);
        const Vertex* b = model.FindVertex(loop.vertices[(i + 1) % loop.vertices.size()]);
        if (!a || !b) return false;
        twice_area += a->position.x * b->position.y - b->position.x * a->position.y;
    }
    if (std::abs(twice_area) < epsilon) return false;
    for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
        const Vertex* a = model.FindVertex(loop.vertices[i]);
        const Vertex* b = model.FindVertex(loop.vertices[(i + 1) % loop.vertices.size()]);
        for (std::size_t j = i + 1; j < loop.vertices.size(); ++j) {
            if (j == (i + 1) % loop.vertices.size() || i == (j + 1) % loop.vertices.size()) continue;
            const Vertex* c = model.FindVertex(loop.vertices[j]);
            const Vertex* d = model.FindVertex(loop.vertices[(j + 1) % loop.vertices.size()]);
            if (a && b && c && d && SegmentsIntersect(a->position, b->position, c->position, d->position))
                return false;
        }
    }
    return true;
}

bool ContainsPoint(const GeometryModel& model, const GeometryLoop& polygon, Vector2d point) {
    bool inside = false;
    for (std::size_t i = 0, j = polygon.vertices.size() - 1; i < polygon.vertices.size(); j = i++) {
        const Vertex* a = model.FindVertex(polygon.vertices[i]);
        const Vertex* b = model.FindVertex(polygon.vertices[j]);
        if (!a || !b) return false;
        if (OnSegment(a->position, b->position, point)) return false;
        const bool spans = (a->position.y > point.y) != (b->position.y > point.y);
        if (spans) {
            const double x = (b->position.x - a->position.x) * (point.y - a->position.y) /
                (b->position.y - a->position.y) + a->position.x;
            if (point.x < x) inside = !inside;
        }
    }
    return inside;
}

bool EntirelyInside(const GeometryModel& model, const GeometryLoop& candidate, const GeometryLoop& border) {
    if (LoopsIntersect(model, candidate, border)) return false;
    return std::all_of(candidate.vertices.begin(), candidate.vertices.end(), [&](VertexId id) {
        const Vertex* vertex = model.FindVertex(id);
        return vertex && ContainsPoint(model, border, vertex->position);
    });
}

} // namespace

GeometryValidation GeometryValidator::Validate(const GeometryModel& model) const {
    GeometryValidation result;
    result.loops.resize(model.Loops().size());
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        result.loops[i].loop_index = i;
        result.loops[i].individually_valid = IsIndividuallyValid(model, model.Loops()[i]);
    }

    std::vector<std::size_t> border_candidates;
    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        const GeometryLoop& loop = model.Loops()[i];
        if (loop.type == EdgeType::Border && result.loops[i].individually_valid) border_candidates.push_back(i);
    }
    std::sort(border_candidates.begin(), border_candidates.end(), [&](std::size_t a, std::size_t b) {
        return model.Loops()[a].completion_order < model.Loops()[b].completion_order;
    });
    if (!border_candidates.empty()) {
        result.active_border_index = border_candidates.front();
        result.loops[*result.active_border_index].active_border = true;
        result.loops[*result.active_border_index].eligible = true;
    }

    for (std::size_t i = 0; i < model.Loops().size(); ++i) {
        const GeometryLoop& loop = model.Loops()[i];
        if (loop.type != EdgeType::Obstacle || !result.loops[i].individually_valid || !result.active_border_index) continue;
        const GeometryLoop& border = model.Loops()[*result.active_border_index];
        LoopValidation& state = result.loops[i];
        state.inside_active_border = EntirelyInside(model, loop, border);
        bool conflict = LoopsIntersect(model, loop, border);
        for (std::size_t j = 0; j < model.Loops().size(); ++j) {
            if (i == j || model.Loops()[j].type != EdgeType::Obstacle ||
                !result.loops[j].individually_valid) continue;
            if (LoopsIntersect(model, loop, model.Loops()[j])) conflict = true;
        }
        state.intersects_conflict = conflict;
        state.eligible = state.inside_active_border && !conflict;
    }
    return result;
}

} // namespace cdt
