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
    /// Signature of the geometry and radius that produced baked_mesh, used to detect outdated output.
    std::string baked_input_signature;
    /// Highest request version already applied; older completions are ignored.
    std::uint64_t applied_request_version{};
    /// Signature most recently submitted, used to avoid duplicate work and undoing a user clear.
    std::string last_requested_signature;

    /// Ignore results older than the latest accepted request version; update status for newer
    /// results and replace the mesh and signature only when the bake succeeds.
    bool Apply(BakeCompletion&& completion);
    /// Clear the displayed mesh and signatures, reset status, and reject work older than the
    /// supplied request version.
    void Reset(std::uint64_t request_version) noexcept;
};

} // namespace cdt
