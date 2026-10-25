#include "app/PathfindingSession.hpp"

namespace cdt {

/// Stores a changed endpoint and invalidates or refreshes the current route as configured.
void PathfindingSession::SetEndpoint(PathEndpoint endpoint, Vector2d position, const NavMesh* mesh, bool auto_update) {
    auto& point = endpoint == PathEndpoint::Start ? start : goal;
    // Ignore unchanged per-frame cursor updates; changed endpoints invalidate and may recompute the route.
    if (point == position) return;
    point = position;
    result.reset();
    UpdateIfEnabled(mesh, auto_update);
}

/// Refreshes the route when a successful bake replaces the accepted navmesh.
void PathfindingSession::OnMeshAccepted(const NavMesh& mesh, bool auto_update) {
    result.reset();
    UpdateIfEnabled(&mesh, auto_update);
}

/// Calculates a route for the current endpoints using the supplied baked mesh.
void PathfindingSession::Generate(const NavMesh& mesh) {
    if (!start || !goal) {
        result.reset();
        return;
    }
    Recompute(mesh);
}

/// Drops the route result when its accepted mesh is removed.
void PathfindingSession::ClearForMeshDeletion() noexcept {
    // The route depends on the removed mesh, but its endpoints can still be reused after another bake.
    result.reset();
}

void PathfindingSession::ClearPath() noexcept {
    // Keep start and goal so the route can be regenerated without placing them again.
    result.reset();
}

void PathfindingSession::Clear() noexcept {
    // Full pathfinding reset used by endpoint clearing and map loading.
    start.reset();
    goal.reset();
    result.reset();
}

/// Recomputes only when auto-update is enabled and a mesh and both endpoints are available.
void PathfindingSession::UpdateIfEnabled(const NavMesh* mesh, bool auto_update) {
    // A route needs auto-update enabled, a baked mesh, and both endpoints.
    if (auto_update && mesh && start && goal) Recompute(*mesh);
}

/// Stores the pathfinder result for the current endpoint pair.
void PathfindingSession::Recompute(const NavMesh& mesh) {
    result = pathfinding::FindPath(mesh, *start, *goal);
}

} // namespace cdt
