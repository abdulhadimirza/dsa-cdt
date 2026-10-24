#include "app/NavmeshBakeCoordinator.hpp"
#include "navmesh/NavmeshPipeline.hpp"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace cdt {
namespace {

using DiagnosticClock = std::chrono::steady_clock;

bool NavmeshDiagnosticsEnabled() {
    const char* value = std::getenv("CDT_NAVMESH_DIAGNOSTICS");
    return value && value[0] != '\0' && std::string_view(value) != "0";
}

double ElapsedMilliseconds(DiagnosticClock::time_point start) {
    return std::chrono::duration<double, std::milli>(DiagnosticClock::now() - start).count();
}

const char* StatusName(GenerationStatus status) {
    switch (status) {
    case GenerationStatus::Success: return "success";
    case GenerationStatus::NoActiveBorder: return "no-active-border";
    case GenerationStatus::Failed: return "failed";
    }
    return "unknown";
}

}

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
    const std::size_t pendingReplacements = pending ? pending->pendingReplacements + 1 : 0;
    pending = Job{generation, std::move(input_key), std::move(geometry), agent_radius, pendingReplacements};
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
        const bool diagnosticsEnabled = NavmeshDiagnosticsEnabled();
        const auto started = diagnosticsEnabled ? DiagnosticClock::now() : DiagnosticClock::time_point{};
        if (diagnosticsEnabled)
            std::clog << "[navmesh-bake] generation=" << job.generation
                << " started pending_replacements=" << job.pendingReplacements << '\n';
        GenerationResult result = NavmeshPipeline{}.Generate(job.geometry, job.agent_radius);
        const double elapsedMs = diagnosticsEnabled ? ElapsedMilliseconds(started) : 0.0;
        const GenerationStatus completedStatus = result.status;
        const std::size_t completedVertices = result.mesh ? result.mesh->Vertices().size() : 0;
        const std::size_t completedTriangles = result.mesh ? result.mesh->Triangles().size() : 0;
        const std::string completedMessage = diagnosticsEnabled ? result.message : std::string{};
        bool newerGenerationPending = false;
        std::uint64_t newerGeneration = 0;
        {
            std::lock_guard lock(mutex);
            running = false;
            newerGenerationPending = pending.has_value();
            if (pending) newerGeneration = pending->generation;
            // Publish as one tagged value for the session to reject stale generations safely.
            completion = BakeCompletion{job.generation, std::move(job.input_key), std::move(result)};
        }
        if (diagnosticsEnabled) {
            std::clog << std::fixed << std::setprecision(3)
                << "[navmesh-bake] generation=" << job.generation
                << " completed status=" << StatusName(completedStatus)
                << " elapsed_ms=" << elapsedMs
                << " output_vertices=" << completedVertices
                << " output_triangles=" << completedTriangles
                << " newer_generation_pending=" << (newerGenerationPending ? "true" : "false");
            if (newerGenerationPending) std::clog << " newer_generation=" << newerGeneration;
            std::clog << " message=" << std::quoted(completedMessage) << '\n';
        }
    }
}

} // namespace cdt
