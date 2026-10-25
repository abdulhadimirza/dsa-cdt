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

// Store the validated geometry, radius, and matching signature in the waiting slot, replacing
// older work that has not started. Its version preserves the worker's completion order.
std::uint64_t NavmeshBakeCoordinator::Request(std::optional<ValidatedNavmeshInput> geometry,
    double agent_radius, std::string input_signature, bool force_full) {
    std::lock_guard lock(mutex);
    // Do not queue new work after shutdown has begun.
    if (stopping) return next_request_version;
    const auto request_version = ++next_request_version;
    // Keep only one request that has not started; a newer request replaces the waiting one.
    pending = Job{request_version, std::move(input_signature), std::move(geometry), agent_radius, force_full};
    wake.notify_one();
    return request_version;
}

// Remove waiting and completed work, then fence any already-running job from publication.
std::uint64_t NavmeshBakeCoordinator::InvalidatePending() {
    std::uint64_t request_version{};
    {
        std::lock_guard lock(mutex);
        // Advancing the version fences running jobs and releases any waiter for an invalidated request.
        request_version = ++next_request_version;
        minimum_valid_request_version = request_version;
        pending.reset();
        completions.clear();
    }
    wake.notify_all();
    return request_version;
}

std::optional<BakeCompletion> NavmeshBakeCoordinator::TakeCompletion() {
    std::lock_guard lock(mutex);
    if (completions.empty()) return std::nullopt;
    auto result = std::move(completions.front());
    completions.pop_front();
    return result;
}

bool NavmeshBakeCoordinator::IsRunning() const {
    std::lock_guard lock(mutex);
    return running || pending.has_value() || !completions.empty();
}

// Wait for requests, generate outside the lock so new UI requests stay responsive, then publish
// the result together with the version and signature of the request that produced it.
void NavmeshBakeCoordinator::Run() {
    NavmeshPipeline pipeline;
    for (;;) {
        // Hold the lock only while waiting for work and taking the newest waiting request.
        Job job;
        {
            std::unique_lock lock(mutex);
            wake.wait(lock, [this] { return stopping || pending.has_value(); });
            if (stopping) return;
            job = std::move(*pending);
            pending.reset();
            running = true;
        }
        // Build the mesh outside the lock so the UI can submit requests while this takes time.
        GenerationResult result = pipeline.Generate(job.geometry, job.agent_radius, job.force_full);
        {
            std::lock_guard lock(mutex);
            running = false;
            // Publish every valid completed snapshot in order, even when newer geometry is queued.
            if (job.request_version >= minimum_valid_request_version)
                completions.push_back(BakeCompletion{job.request_version, std::move(job.input_signature), std::move(result)});
        }
        wake.notify_all();
    }
}

} // namespace cdt
