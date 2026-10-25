#pragma once

#include "cdt/ConstrainedDelaunay.hpp"
#include "validation/GeometryValidator.hpp"

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace cdt {

/// Worker output tagged with its request version and the input signature used to produce it.
struct BakeCompletion {
    /// Request version lets the UI ignore a result when a newer request replaced it.
    std::uint64_t request_version{};
    /// Signature identifies the geometry and radius used to make result.
    std::string input_signature;
    /// Outcome and generated mesh returned by the background worker.
    GenerationResult result;
};

class NavmeshBakeCoordinator {
public:
    /// Start the single background worker, which waits until a request is queued.
    NavmeshBakeCoordinator();
    /// Stop accepting work, discard a pending request, and join the worker before destruction.
    ~NavmeshBakeCoordinator();
    NavmeshBakeCoordinator(const NavmeshBakeCoordinator&) = delete;
    NavmeshBakeCoordinator& operator=(const NavmeshBakeCoordinator&) = delete;

    /// Replace the waiting request with newer validated input and return its request version.
    std::uint64_t Request(std::optional<ValidatedNavmeshInput> geometry,
        double agent_radius, std::string input_signature);
    /// Discards queued and completed work and returns a version that invalidates running work.
    [[nodiscard]] std::uint64_t InvalidatePending();
    
    /// Take and clear the most recently completed bake, if one is available.
    [[nodiscard]] std::optional<BakeCompletion> TakeCompletion();

    /// Whether a job is currently running or waiting in the single pending slot.
    [[nodiscard]] bool IsRunning() const;

private:
    /// Copied input values let the worker build a mesh without reading the editable model.
    struct Job {
        /// Version assigned when this request entered the coordinator.
        std::uint64_t request_version{};
        /// Signature carried through to the result so it can be compared with current inputs.
        std::string input_signature;
        /// Validated geometry copied when the request was made.
        std::optional<ValidatedNavmeshInput> geometry;
        /// Radius copied with the geometry for this bake.
        double agent_radius{};
    };
    /// Worker loop: wait for a job, run the pipeline without holding the lock, then publish completion.
    void Run();
    mutable std::mutex mutex;
    std::condition_variable wake;
    /// Newest request waiting for the worker; newer requests replace this slot.
    std::optional<Job> pending;
    /// Most recently completed request waiting for the UI to collect it.
    std::optional<BakeCompletion> completion;
    std::thread worker;
    /// Next number used to order requests and reject results from older work.
    std::uint64_t next_request_version{0};
    /// True while the worker is generating a mesh for a request.
    bool running{false};
    /// Tells the waiting worker to stop and exit.
    bool stopping{false};
};

} // namespace cdt
