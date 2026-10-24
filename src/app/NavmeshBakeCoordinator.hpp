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

/// Worker output tagged with its request generation and the input identity used to produce it.
struct BakeCompletion {
    std::uint64_t generation{};
    std::string input_key;
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

    /// Replace the pending slot with a newer snapshot and return its monotonically increasing generation.
    std::uint64_t Request(std::optional<ValidatedNavmeshInput> geometry,
        double agent_radius, std::string input_key);
    
    /// Take and clear the most recently completed bake, if one is available.
    [[nodiscard]] std::optional<BakeCompletion> TakeCompletion();

    /// Whether a job is currently running or waiting in the single pending slot.
    [[nodiscard]] bool IsRunning() const;

private:
    /// Captured immutable inputs let the worker generate without accessing the editable model.
    struct Job {
        std::uint64_t generation{};
        std::string input_key;
        std::optional<ValidatedNavmeshInput> geometry;
        double agent_radius{};
    };
    /// Worker loop: wait for a job, run the pipeline without holding the lock, then publish completion.
    void Run();
    mutable std::mutex mutex;
    std::condition_variable wake;
    std::optional<Job> pending;
    std::optional<BakeCompletion> completion;
    std::thread worker;
    std::uint64_t next_generation{0};
    bool running{false};
    bool stopping{false};
};

} // namespace cdt
