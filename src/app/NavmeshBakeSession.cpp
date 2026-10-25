#include "app/NavmeshBakeSession.hpp"
#include <utility>

namespace cdt {

// Ignore results from older requests; otherwise record the status and, on success, keep the mesh
// paired with the signature of the inputs that produced it. A failed bake leaves the prior mesh.
bool NavmeshBakeSession::Apply(BakeCompletion&& completion) {
    // Request versions make completion order safe when newer requests supersede in-flight work.
    if (completion.request_version <= applied_request_version) return false;
    applied_request_version = completion.request_version;
    bake_attempted = true;
    last_bake_status = completion.result.status;
    if (completion.result.status == GenerationStatus::Success) {
        // Keep the prior successful mesh on failure, along with its matching signature.
        baked_mesh = std::move(completion.result.mesh);
        baked_input_signature = std::move(completion.input_signature);
    }
    return true;
}

// Clear all displayed bake state and set the minimum acceptable request version so older worker
// results cannot restore a mesh after a clear or map load.
void NavmeshBakeSession::Reset(std::uint64_t request_version) noexcept {
    baked_mesh.reset();
    bake_attempted = false;
    last_bake_status = GenerationStatus::Failed;
    baked_input_signature.clear();
    applied_request_version = request_version;
    last_requested_signature.clear();
}

} // namespace cdt
