#include "app/NavmeshBakeSession.hpp"
#include <utility>

namespace cdt {

bool NavmeshBakeSession::Apply(BakeCompletion&& completion) {
    // Generation numbers make completion order safe when newer requests supersede in-flight work.
    if (completion.generation <= applied_generation) return false;
    applied_generation = completion.generation;
    bake_attempted = true;
    last_bake_status = completion.result.status;
    if (completion.result.status == GenerationStatus::Success) {
        // Keep the prior successful mesh on failure, along with its matching key.
        baked_mesh = std::move(completion.result.mesh);
        baked_input_key = std::move(completion.input_key);
    }
    return true;
}

} // namespace cdt
