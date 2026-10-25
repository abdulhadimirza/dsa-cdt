#pragma once

#include "app/NavmeshBakeCoordinator.hpp"
#include <cstdint>
#include <optional>
#include <string>

namespace cdt {

/// UI-facing bake state; asynchronous work and synchronization remain in the coordinator.
struct NavmeshBakeSession {
    /// Most recently accepted successful mesh; failures leave the last good mesh visible.
    std::optional<NavMesh> baked_mesh;
    /// True after a manual or automatic request has been issued or completed.
    bool bake_attempted{false};
    /// Status of the newest completion accepted by Apply.
    GenerationStatus last_bake_status{GenerationStatus::Failed};
    /// Input identity associated with baked_mesh, used to detect stale output.
    std::string baked_input_key;
    /// Highest completion generation already applied; older completions are ignored.
    std::uint64_t applied_generation{};
    /// Input key last requested manually or automatically; also prevents auto-bake from undoing a user clear.
    std::string last_requested_key;

    /// Ignore stale completions; accept newer status and replace mesh/key only on success.
    bool Apply(BakeCompletion&& completion);
    /// Clears mesh state and rejects any asynchronous work issued before generation.
    void Reset(std::uint64_t generation) noexcept;
};

} // namespace cdt
