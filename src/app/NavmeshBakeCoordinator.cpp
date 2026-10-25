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
// older work that has not started. Return its version so the session can reject older results.
std::uint64_t NavmeshBakeCoordinator::Request(std::optional<ValidatedNavmeshInput> geometry,
    double agent_radius, std::string input_signature) {
    std::lock_guard lock(mutex);
    // Do not queue new work after shutdown has begun.
    if (stopping) return next_request_version;
    const auto request_version = ++next_request_version;
        // Keep only one request that has not started; a newer request replaces the waiting one.
    pending = Job{request_version, std::move(input_signature), std::move(geometry), agent_radius};
    wake.notify_one();
    return request_version;
}

// Remove waiting and completed work, then advance the version so an already-running job's result
// will be recognized as outdated when it reaches the session.
std::uint64_t NavmeshBakeCoordinator::InvalidatePending() {
    std::lock_guard lock(mutex);
    // Advancing the request version makes running completions rejectable by NavmeshBakeSession::Apply.
    const auto request_version = ++next_request_version;
    pending.reset();
    completion.reset();
    return request_version;
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

// Wait for requests, generate outside the lock so new UI requests stay responsive, then publish
// the result together with the version and signature of the request that produced it.
void NavmeshBakeCoordinator::Run() {
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
        GenerationResult result = NavmeshPipeline{}.Generate(job.geometry, job.agent_radius);
        {
            std::lock_guard lock(mutex);
            running = false;
            // Publish the result with its request version so outdated work can be rejected safely.
            completion = BakeCompletion{job.request_version, std::move(job.input_signature), std::move(result)};
        }
    }
}

} // namespace cdt
