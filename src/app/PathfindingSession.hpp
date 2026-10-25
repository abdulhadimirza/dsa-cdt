#pragma once

#include "core/Vector2d.hpp"
#include "pathfinding/Pathfinder.hpp"
#include "pathfinding/PathfindingView.hpp"

#include <optional>

namespace cdt {

/// Selects which stored path endpoint an app-level navigation action updates.
enum class PathEndpoint { Start, Goal };

/// Owns navigation endpoints and route updates computed from accepted baked meshes.
class PathfindingSession {
public:
    /// Returns a non-owning snapshot for UI and renderer consumption.
    [[nodiscard]] pathfinding::PathfindingView View() const noexcept { return {start, goal, result}; }

    /// Stores a changed endpoint, clears its stale result, and optionally recomputes against the accepted mesh.
    void SetEndpoint(PathEndpoint endpoint, Vector2d position, const NavMesh* mesh, bool auto_update);
    /// Clears the prior result and optionally computes a route against a newly accepted mesh.
    void OnMeshAccepted(const NavMesh& mesh, bool auto_update);
    /// Computes a route on demand when both endpoints are set.
    void Generate(const NavMesh& mesh);
    /// Clears only the generated result, preserving the two endpoints.
    void ClearPath() noexcept;
    /// Clears the generated result and both endpoints.
    void Clear() noexcept;
    /// Clears the displayed route after the accepted mesh is deleted.
    void ClearForMeshDeletion() noexcept;

private:
    /// Recomputes only when auto-update is enabled and inputs are complete.
    void UpdateIfEnabled(const NavMesh* mesh, bool auto_update);
    /// Replaces the stored result with the pathfinder's result for current endpoints.
    void Recompute(const NavMesh& mesh);

    std::optional<Vector2d> start;
    std::optional<Vector2d> goal;
    std::optional<pathfinding::PathResult> result;
};

} // namespace cdt
