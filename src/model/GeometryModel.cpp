#include "model/GeometryModel.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace cdt {
namespace {

double Cross(Vector2d a, Vector2d b, Vector2d c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool OnSegment(Vector2d a, Vector2d b, Vector2d point) {
    constexpr double epsilon = 1e-9;
    return std::abs(Cross(a, b, point)) <= epsilon &&
        point.x >= std::min(a.x, b.x) - epsilon && point.x <= std::max(a.x, b.x) + epsilon &&
        point.y >= std::min(a.y, b.y) - epsilon && point.y <= std::max(a.y, b.y) + epsilon;
}

bool SegmentsIntersect(Vector2d a, Vector2d b, Vector2d c, Vector2d d) {
    constexpr double epsilon = 1e-9;
    const double ab_c = Cross(a, b, c);
    const double ab_d = Cross(a, b, d);
    const double cd_a = Cross(c, d, a);
    const double cd_b = Cross(c, d, b);
    if (((ab_c > epsilon && ab_d < -epsilon) || (ab_c < -epsilon && ab_d > epsilon)) &&
        ((cd_a > epsilon && cd_b < -epsilon) || (cd_a < -epsilon && cd_b > epsilon))) return true;
    return OnSegment(a, b, c) || OnSegment(a, b, d) || OnSegment(c, d, a) || OnSegment(c, d, b);
}

} // namespace

VertexId GeometryModel::AddVertex(Vector2d position) {
    const VertexId id = next_id++;
    vertices.push_back(Vertex{id, position});
    return id;
}

bool GeometryModel::MoveVertex(VertexId id, Vector2d position) {
    const auto it = std::find_if(vertices.begin(), vertices.end(), [id](const Vertex& vertex) {
        return vertex.id == id;
    });
    if (it == vertices.end()) return false;
    it->position = position;
    RebuildLoops();
    return true;
}

bool GeometryModel::MoveVerticesBy(const std::vector<VertexId>& ids, Vector2d offset) {
    bool moved = false;
    for (Vertex& vertex : vertices) {
        if (std::find(ids.begin(), ids.end(), vertex.id) == ids.end()) continue;
        vertex.position = vertex.position + offset;
        moved = true;
    }
    if (moved) RebuildLoops();
    return moved;
}

std::vector<VertexId> GeometryModel::LoopGroupForVertex(VertexId id) const {
    if (!FindVertex(id)) return {};

    std::vector<std::vector<VertexId>> cyclic_components;
    for (const EdgeType type : {EdgeType::Border, EdgeType::Obstacle}) {
        std::unordered_map<VertexId, std::vector<VertexId>> adjacency;
        for (const GeometryEdge& edge : edges) {
            if (edge.type != type) continue;
            adjacency[edge.first].push_back(edge.second);
            adjacency[edge.second].push_back(edge.first);
        }

        std::unordered_map<VertexId, int> discovery;
        std::unordered_map<VertexId, int> low;
        std::vector<std::pair<VertexId, VertexId>> edge_stack;
        int time = 0;
        std::function<void(VertexId, VertexId)> visit = [&](VertexId current, VertexId parent) {
            discovery[current] = low[current] = ++time;
            for (const VertexId next : adjacency[current]) {
                if (!discovery.contains(next)) {
                    edge_stack.emplace_back(current, next);
                    visit(next, current);
                    low[current] = std::min(low[current], low[next]);
                    if (low[next] >= discovery[current]) {
                        std::unordered_set<VertexId> component;
                        std::size_t component_edges = 0;
                        while (!edge_stack.empty()) {
                            const auto edge = edge_stack.back();
                            edge_stack.pop_back();
                            component.insert(edge.first);
                            component.insert(edge.second);
                            ++component_edges;
                            if (edge.first == current && edge.second == next) break;
                        }
                        if (component_edges >= component.size())
                            cyclic_components.emplace_back(component.begin(), component.end());
                    }
                } else if (next != parent && discovery[next] < discovery[current]) {
                    edge_stack.emplace_back(current, next);
                    low[current] = std::min(low[current], discovery[next]);
                }
            }
        };

        for (const auto& [vertex, neighbors] : adjacency) {
            (void)neighbors;
            if (!discovery.contains(vertex)) visit(vertex, vertex);
        }
    }

    std::unordered_set<VertexId> group{id};
    bool changed = true;
    while (changed) {
        changed = false;
        for (const auto& component : cyclic_components) {
            const bool touches_group = std::any_of(component.begin(), component.end(),
                [&group](VertexId vertex) { return group.contains(vertex); });
            if (!touches_group) continue;
            for (const VertexId vertex : component) changed = group.insert(vertex).second || changed;
        }
    }
    if (group.size() == 1) return {};
    return {group.begin(), group.end()};
}

bool GeometryModel::RemoveVertex(VertexId id) {
    const auto it = std::find_if(vertices.begin(), vertices.end(), [id](const Vertex& vertex) {
        return vertex.id == id;
    });
    if (it == vertices.end()) return false;
    edges.erase(std::remove_if(edges.begin(), edges.end(), [id](const GeometryEdge& edge) {
        return edge.first == id || edge.second == id;
    }), edges.end());
    vertices.erase(it);
    RebuildLoops();
    return true;
}

bool GeometryModel::AddEdge(VertexId first, VertexId second, EdgeType type) {
    if (first == second || !FindVertex(first) || !FindVertex(second)) return false;
    const auto duplicate = std::find_if(edges.begin(), edges.end(), [=](const GeometryEdge& edge) {
        return edge.type == type && ((edge.first == first && edge.second == second) ||
            (edge.first == second && edge.second == first));
    });
    if (duplicate != edges.end()) return false;
    edges.push_back(GeometryEdge{first, second, type});
    RebuildLoops();
    return true;
}

void GeometryModel::Clear() noexcept {
    vertices.clear();
    edges.clear();
    loops.clear();
}

const Vertex* GeometryModel::FindVertex(VertexId id) const noexcept {
    const auto it = std::find_if(vertices.begin(), vertices.end(), [id](const Vertex& vertex) {
        return vertex.id == id;
    });
    return it == vertices.end() ? nullptr : &*it;
}

void GeometryModel::RebuildLoops() {
    loops.clear();
    for (const EdgeType type : {EdgeType::Border, EdgeType::Obstacle}) {
        std::unordered_map<VertexId, std::vector<VertexId>> adjacency;
        for (const GeometryEdge& edge : edges) {
            if (edge.type != type) continue;
            adjacency[edge.first].push_back(edge.second);
            adjacency[edge.second].push_back(edge.first);
        }

        std::unordered_set<VertexId> visited;
        for (const auto& [start, neighbors] : adjacency) {
            (void)neighbors;
            if (visited.contains(start)) continue;
            std::vector<VertexId> component;
            std::vector<VertexId> pending{start};
            bool is_cycle = true;
            while (!pending.empty()) {
                const VertexId current = pending.back();
                pending.pop_back();
                if (!visited.insert(current).second) continue;
                component.push_back(current);
                const auto& adjacent = adjacency[current];
                if (adjacent.size() != 2) is_cycle = false;
                for (const VertexId next : adjacent) if (!visited.contains(next)) pending.push_back(next);
            }
            if (!is_cycle || component.size() < 3) continue;

            std::vector<VertexId> ordered;
            ordered.reserve(component.size());
            VertexId previous = -1; // Relies on id -1 being impossible (which it is at the time of writing)
            VertexId current = start;
            do {
                ordered.push_back(current);
                const auto& adjacent = adjacency[current];
                const VertexId next = adjacent[0] == previous ? adjacent[1] : adjacent[0];
                previous = current;
                current = next;
            } while (current != start && ordered.size() <= component.size());
            if (current != start || ordered.size() != component.size()) continue;

            double twice_area = 0.0;
            bool valid = true;
            for (std::size_t i = 0; i < ordered.size(); ++i) {
                const Vertex* a = FindVertex(ordered[i]);
                const Vertex* b = FindVertex(ordered[(i + 1) % ordered.size()]);
                if (!a || !b) { valid = false; break; }
                twice_area += a->position.x * b->position.y - b->position.x * a->position.y;
            }
            if (std::abs(twice_area) < 1e-9) valid = false;
            if (twice_area < 0.0) std::reverse(ordered.begin(), ordered.end());

            for (std::size_t i = 0; valid && i < ordered.size(); ++i) {
                const Vertex* a = FindVertex(ordered[i]);
                const Vertex* b = FindVertex(ordered[(i + 1) % ordered.size()]);
                for (std::size_t j = i + 1; j < ordered.size(); ++j) {
                    if (j == i || j == (i + 1) % ordered.size() || i == (j + 1) % ordered.size()) continue;
                    const Vertex* c = FindVertex(ordered[j]);
                    const Vertex* d = FindVertex(ordered[(j + 1) % ordered.size()]);
                    if (a && b && c && d && SegmentsIntersect(a->position, b->position, c->position, d->position)) {
                        valid = false;
                        break;
                    }
                }
            }
            loops.push_back(GeometryLoop{type, std::move(ordered), valid});
        }
    }
}

} // namespace cdt
