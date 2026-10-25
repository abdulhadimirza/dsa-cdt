#include "core/GeometryPredicates.hpp"
#include "model/GeometryModel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <tuple>
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

// Calculates twice the signed polygon area in loop order; the shared origin-relative sum
// reduces cancellation when coordinates are large relative to the polygon's dimensions.
double SignedTwiceArea(const std::vector<VertexId>& ordered,
    const std::unordered_map<VertexId, Vector2d>& positions) {
    std::vector<Vector2d> polygon;
    polygon.reserve(ordered.size());
    for (VertexId id : ordered) polygon.push_back(positions.at(id));
    return geometry::SignedTwiceArea(polygon);
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

// Add one vertex at the supplied position, advance the model version, and return its new ID.
VertexId GeometryModel::AddVertex(Vector2d position) {
    const VertexId id = next_id++;
    vertices.push_back(Vertex{id, position});
    AdvanceVersion();
    return id;
}

// Move the named vertex and rebuild its closed loops; report false if it is missing, and leave the
// version unchanged when the requested position already matches the stored position.
bool GeometryModel::MoveVertex(VertexId id, Vector2d position) {
    // Locate the vertex whose position the caller wants to change.
    const auto it = std::find_if(vertices.begin(), vertices.end(), [id](const Vertex& vertex) {
        return vertex.id == id;
    });
    if (it == vertices.end()) return false;
    // Avoid rebuilding loops or changing the version for a drag update with no movement.
    if (it->position == position) return true;
    it->position = position;
    RebuildLoops();
    AdvanceVersion();
    return true;
}

// Move every vertex whose ID is listed, then rebuild loops and advance the version when a
// non-zero offset changed at least one selected vertex. Return whether any selected ID existed.
bool GeometryModel::MoveVerticesBy(const std::vector<VertexId>& ids, Vector2d offset) {
    bool moved = false;
    for (Vertex& vertex : vertices) {
        if (std::find(ids.begin(), ids.end(), vertex.id) == ids.end()) continue;
        vertex.position = vertex.position + offset;
        moved = true;
    }
    // A zero offset can find selected vertices without changing their geometry.
    if (moved && offset != Vector2d{}) {
        RebuildLoops();
        AdvanceVersion();
    }
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

// Remove the named vertex and all edges attached to it, rebuild loops, and advance the version.
// Return false without changing the model when the vertex does not exist.
bool GeometryModel::RemoveVertex(VertexId id) {
    // Locate the vertex first so a missing ID leaves all model data untouched.
    const auto it = std::find_if(vertices.begin(), vertices.end(), [id](const Vertex& vertex) {
        return vertex.id == id;
    });
    if (it == vertices.end()) return false;
    // Remove every edge attached to the vertex before removing the vertex itself.
    edges.erase(std::remove_if(edges.begin(), edges.end(), [id](const GeometryEdge& edge) {
        return edge.first == id || edge.second == id;
    }), edges.end());
    vertices.erase(it);
    RebuildLoops();
    AdvanceVersion();
    return true;
}

// Add an edge when both endpoints exist and that edge is not already present, even in reverse;
// rebuild loops and advance the version only after the edge is accepted.
bool GeometryModel::AddEdge(VertexId first, VertexId second, EdgeType type) {
    if (first == second || !FindVertex(first) || !FindVertex(second)) return false;
    // Treat reversed endpoints as the same undirected edge when checking for duplicates.
    const auto duplicate = std::find_if(edges.begin(), edges.end(), [=](const GeometryEdge& edge) {
        return edge.type == type && ((edge.first == first && edge.second == second) ||
            (edge.first == second && edge.second == first));
    });
    if (duplicate != edges.end()) return false;
    edges.push_back(GeometryEdge{first, second, type});
    RebuildLoops();
    AdvanceVersion();
    return true;
}

// Validate the complete incoming vertex and edge lists before replacing the model. Invalid input
// leaves the current geometry and version untouched; accepted input rebuilds loops and advances it.
bool GeometryModel::Replace(std::vector<Vertex> new_vertices, std::vector<GeometryEdge> new_edges) {
    std::unordered_set<VertexId> ids;
    VertexId new_next_id = 0;
    // Check every position and ID before changing the current model.
    for (const Vertex& vertex : new_vertices) {
        if (!std::isfinite(vertex.position.x) || !std::isfinite(vertex.position.y) ||
            vertex.id == std::numeric_limits<VertexId>::max() || !ids.insert(vertex.id).second) return false;
        new_next_id = std::max(new_next_id, vertex.id + 1);
    }

    std::set<std::tuple<VertexId, VertexId, EdgeType>> unique_edges;
    // Require valid, distinct endpoints and reject duplicate edges in either endpoint order.
    for (const GeometryEdge& edge : new_edges) {
        if (edge.first == edge.second || !ids.contains(edge.first) || !ids.contains(edge.second) ||
            (edge.type != EdgeType::Border && edge.type != EdgeType::Obstacle)) return false;
        const auto endpoints = std::minmax(edge.first, edge.second);
        if (!unique_edges.emplace(endpoints.first, endpoints.second, edge.type).second) return false;
    }

    // Commit only after all input checks pass, then rebuild derived loops for the accepted data.
    vertices = std::move(new_vertices);
    edges = std::move(new_edges);
    loops.clear();
    next_id = new_next_id;
    next_loop_completion_order = 0;
    RebuildLoops();
    AdvanceVersion();
    return true;
}

// Remove all vertices, edges, and loops, reset loop ordering, and advance the version even when
// the model is already empty so callers can treat every Clear request as a cache invalidation.
void GeometryModel::Clear() noexcept {
    vertices.clear();
    edges.clear();
    loops.clear();
    next_loop_completion_order = 0;
    AdvanceVersion();
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
