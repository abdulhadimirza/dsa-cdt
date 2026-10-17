#include "model/GeometryModel.hpp"

#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace cdt {

namespace {

using Adjacency = std::unordered_map<VertexId, std::vector<VertexId>>;

// Builds an undirected adjacency list containing only edges of the requested type.
Adjacency BuildAdjacency(const std::vector<GeometryEdge>& edges, EdgeType type) {
    Adjacency adjacency;
    for (const GeometryEdge& edge : edges) {
        if (edge.type != type) continue;
        // Store each undirected edge in both directions for graph traversal.
        adjacency[edge.first].push_back(edge.second);
        adjacency[edge.second].push_back(edge.first);
    }
    return adjacency;
}

// Orders a degree-two connected component into a closed cycle starting at start.
bool OrderCycle(VertexId start, const Adjacency& adjacency,
    const std::vector<VertexId>& component, std::vector<VertexId>& ordered) {
    ordered.clear();
    ordered.reserve(component.size());
    VertexId previous{};
    bool has_previous = false;
    VertexId current = start;
    do {
        ordered.push_back(current);
        const auto& adjacent = adjacency.at(current);
        // At each degree-two vertex, continue along the edge not just traversed.
        const VertexId next = has_previous && adjacent[0] == previous ? adjacent[1] : adjacent[0];
        previous = current;
        has_previous = true;
        current = next;
    } while (current != start && ordered.size() <= component.size());
    return current == start && ordered.size() == component.size();
}

// Calculates twice the signed polygon area for the ordered vertex IDs.
double SignedTwiceArea(const std::vector<VertexId>& ordered,
    const std::unordered_map<VertexId, Vector2d>& positions) {
    double twice_area = 0.0;
    for (std::size_t i = 0; i < ordered.size(); ++i) {
        const Vector2d& a = positions.at(ordered[i]);
        const Vector2d& b = positions.at(ordered[(i + 1) % ordered.size()]);
        twice_area += a.x * b.y - b.x * a.y;
    }
    return twice_area;
}

// Creates a stable key from the edge type and the unordered set of loop vertices.
std::string LoopKey(EdgeType type, const std::vector<VertexId>& vertices) {
    std::vector<VertexId> sorted_vertices = vertices;
    std::sort(sorted_vertices.begin(), sorted_vertices.end());
    std::string key = std::to_string(static_cast<int>(type)) + ":";
    for (const VertexId id : sorted_vertices) key += std::to_string(id) + ",";
    return key;
}

} // namespace

// Adds a vertex and returns its newly assigned ID.
VertexId GeometryModel::AddVertex(Vector2d position) {
    const VertexId id = next_id++;
    vertices.push_back(Vertex{id, position});
    return id;
}

// Moves a vertex, rebuilding derived loops when the vertex exists.
bool GeometryModel::MoveVertex(VertexId id, Vector2d position) {
    const auto it = std::find_if(vertices.begin(), vertices.end(), [id](const Vertex& vertex) {
        return vertex.id == id;
    });
    if (it == vertices.end()) return false;
    it->position = position;
    RebuildLoops();
    return true;
}

// Moves each requested vertex by an offset and rebuilds loops if anything moved.
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

// Returns every vertex connected to the given vertex, or an empty list if it has no chain.
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

// Removes a vertex and its incident edges, then rebuilds derived loops.
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

// Adds an edge if its endpoints are valid and the undirected edge is not already present.
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

// Removes all geometry and resets loop completion ordering.
void GeometryModel::Clear() noexcept {
    vertices.clear();
    edges.clear();
    loops.clear();
    next_loop_completion_order = 0;
}

// Finds a vertex by ID, returning nullptr when the ID is not present.
const Vertex* GeometryModel::FindVertex(VertexId id) const noexcept {
    const auto it = std::find_if(vertices.begin(), vertices.end(), [id](const Vertex& vertex) {
        return vertex.id == id;
    });
    return it == vertices.end() ? nullptr : &*it;
}

// Recomputes closed loops from edges while preserving completion order for existing loops.
void GeometryModel::RebuildLoops() {
    std::unordered_map<std::string, std::uint64_t> previous_orders;
    for (const GeometryLoop& loop : loops) {
        // Keep completion identity when rebuilding the same loop after a geometry edit.
        previous_orders.emplace(LoopKey(loop.type, loop.vertices), loop.completion_order);
    }
    loops.clear();
    // Cache positions so area calculation can use direct lookups for valid edge endpoints.
    const std::unordered_map<VertexId, Vector2d> positions = [&] {
        std::unordered_map<VertexId, Vector2d> result;
        result.reserve(vertices.size());
        for (const Vertex& vertex : vertices) result.emplace(vertex.id, vertex.position);
        return result;
    }();
    for (const EdgeType type : {EdgeType::Border, EdgeType::Obstacle}) {
        // Build separate graphs so touching edges of different types cannot form one loop.
        const Adjacency adjacency = BuildAdjacency(edges, type);
        std::unordered_set<VertexId> visited;
        for (const auto& [start, neighbors] : adjacency) {
            (void)neighbors;
            if (visited.contains(start)) continue;

            // Gather one connected component and require every vertex to have degree two.
            std::vector<VertexId> component;
            std::vector<VertexId> pending{start};
            bool is_cycle = true;
            while (!pending.empty()) {
                const VertexId current = pending.back();
                pending.pop_back();
                if (!visited.insert(current).second) continue;
                component.push_back(current);
                const auto& adjacent = adjacency.at(current);
                if (adjacent.size() != 2) is_cycle = false;
                for (const VertexId next : adjacent) if (!visited.contains(next)) pending.push_back(next);
            }
            if (!is_cycle || component.size() < 3) continue;

            std::vector<VertexId> ordered;
            if (!OrderCycle(start, adjacency, component, ordered)) continue;
            // Normalize winding so all returned loops have counterclockwise vertex order.
            if (SignedTwiceArea(ordered, positions) < 0.0) std::reverse(ordered.begin(), ordered.end());

            const std::string key = LoopKey(type, ordered);
            const auto prior = previous_orders.find(key);
            // Assign a completion order only to loops that did not exist before this rebuild.
            const std::uint64_t completion_order = prior == previous_orders.end()
                ? next_loop_completion_order++ : prior->second;
            loops.push_back(GeometryLoop{type, std::move(ordered), completion_order});
        }
    }
    // Keep public loop order stable according to when each loop was completed.
    std::sort(loops.begin(), loops.end(), [](const GeometryLoop& a, const GeometryLoop& b) {
        return a.completion_order < b.completion_order;
    });
}

} // namespace cdt
