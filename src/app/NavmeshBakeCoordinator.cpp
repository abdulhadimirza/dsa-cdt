#include "app/NavmeshBakeCoordinator.hpp"
#include "navmesh/NavmeshPipeline.hpp"

namespace cdt {

// Launch the worker only after synchronization members have been constructed.
NavmeshBakeCoordinator::NavmeshBakeCoordinator() : worker(&NavmeshBakeCoordinator::Run, this) {}

NavmeshBakeCoordinator::~NavmeshBakeCoordinator() {
    // Signal shutdown and discard queued work; an already-running bake completes before join returns.
    {
        std::lock_guard lock(mutex);
        stopping = true;
        pending.reset();
    }
    wake.notify_one();
    if (worker.joinable()) worker.join();
}

std::uint64_t NavmeshBakeCoordinator::Request(std::optional<ValidatedNavmeshInput> geometry,
    double agent_radius, std::string input_key) {
    std::lock_guard lock(mutex);
    if (stopping) return next_generation;
    const auto generation = ++next_generation;
    // A single pending slot coalesces rapid edits: the worker processes the newest requested state.
    pending = Job{generation, std::move(input_key), std::move(geometry), agent_radius};
    wake.notify_one();
    return generation;
}

std::optional<BakeCompletion> NavmeshBakeCoordinator::TakeCompletion() {
    std::lock_guard lock(mutex);
    auto result = std::move(completion);
    completion.reset();
    return result;
}

bool NavmeshBakeCoordinator::IsRunning() const {
    std::lock_guard lock(mutex);
    return running || pending.has_value();
}

void NavmeshBakeCoordinator::Run() {
    for (;;) {
        Job job;
        {
            std::unique_lock lock(mutex);
            wake.wait(lock, [this] { return stopping || pending.has_value(); });
            if (stopping) return;
            job = std::move(*pending);
            pending.reset();
            running = true;
        }
        // Generation is intentionally outside the mutex so UI requests remain responsive.
        GenerationResult result = NavmeshPipeline{}.Generate(job.geometry, job.agent_radius);
        {
            std::lock_guard lock(mutex);
            running = false;
            // Publish as one tagged value for the session to reject stale generations safely.
            completion = BakeCompletion{job.generation, std::move(job.input_key), std::move(result)};
        }
    }
}

} // namespace cdt
