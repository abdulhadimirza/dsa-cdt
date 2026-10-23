#pragma once

#include "cdt/ConstrainedDelaunay.hpp"

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace cdt {

struct BakeCompletion {
    std::uint64_t generation{};
    std::string input_key;
    GenerationResult result;
};

class NavmeshBakeCoordinator {
public:
    NavmeshBakeCoordinator();
    ~NavmeshBakeCoordinator();
    NavmeshBakeCoordinator(const NavmeshBakeCoordinator&) = delete;
    NavmeshBakeCoordinator& operator=(const NavmeshBakeCoordinator&) = delete;

    std::uint64_t Request(const GeometryModel& model, const GeometryValidation& validation,
        double agent_radius, std::string input_key);
    [[nodiscard]] std::optional<BakeCompletion> TakeCompletion();
    [[nodiscard]] bool IsRunning() const;

private:
    struct Job {
        std::uint64_t generation{};
        std::string input_key;
        GeometryModel model;
        GeometryValidation validation;
        double agent_radius{};
    };
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
