#include "model/GeometryModel.hpp"

#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace cdt {

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

std::vector<VertexId> GeometryModel::ChainForVertex(VertexId id) const {
    if (!FindVertex(id)) return {};

    std::unordered_map<VertexId, std::vector<VertexId>> adjacency;
    for (const GeometryEdge& edge : edges) {
        adjacency[edge.first].push_back(edge.second);
        adjacency[edge.second].push_back(edge.first);
    }

    const auto start = adjacency.find(id);
    if (start == adjacency.end()) return {};

    std::vector<VertexId> chain;
    std::unordered_set<VertexId> visited{id};
    std::vector<VertexId> pending{id};
    while (!pending.empty()) {
        const VertexId current = pending.back();
        pending.pop_back();
        chain.push_back(current);
        for (const VertexId next : adjacency[current]) {
            if (visited.insert(next).second) pending.push_back(next);
        }
    }
    return chain;
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
    next_loop_completion_order = 0;
}

const Vertex* GeometryModel::FindVertex(VertexId id) const noexcept {
    const auto it = std::find_if(vertices.begin(), vertices.end(), [id](const Vertex& vertex) {
        return vertex.id == id;
    });
    return it == vertices.end() ? nullptr : &*it;
}

void GeometryModel::RebuildLoops() {
    std::unordered_map<std::string, std::uint64_t> previous_orders;
    for (const GeometryLoop& loop : loops) {
        std::vector<VertexId> key_vertices = loop.vertices;
        std::sort(key_vertices.begin(), key_vertices.end());
        std::string key = std::to_string(static_cast<int>(loop.type)) + ":";
        for (const VertexId id : key_vertices) key += std::to_string(id) + ",";
        previous_orders.emplace(std::move(key), loop.completion_order);
    }
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
            for (std::size_t i = 0; i < ordered.size(); ++i) {
                const Vertex* a = FindVertex(ordered[i]);
                const Vertex* b = FindVertex(ordered[(i + 1) % ordered.size()]);
                if (a && b) twice_area += a->position.x * b->position.y - b->position.x * a->position.y;
            }
            if (twice_area < 0.0) std::reverse(ordered.begin(), ordered.end());

            std::vector<VertexId> key_vertices = ordered;
            std::sort(key_vertices.begin(), key_vertices.end());
            std::string key = std::to_string(static_cast<int>(type)) + ":";
            for (const VertexId id : key_vertices) key += std::to_string(id) + ",";
            const auto prior = previous_orders.find(key);
            const std::uint64_t completion_order = prior == previous_orders.end()
                ? next_loop_completion_order++ : prior->second;
            loops.push_back(GeometryLoop{type, std::move(ordered), completion_order});
        }
    }
    std::sort(loops.begin(), loops.end(), [](const GeometryLoop& a, const GeometryLoop& b) {
        return a.completion_order < b.completion_order;
    });
}

} // namespace cdt
