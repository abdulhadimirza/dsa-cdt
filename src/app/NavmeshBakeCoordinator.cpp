#include "app/NavmeshBakeCoordinator.hpp"

namespace cdt {

NavmeshBakeCoordinator::NavmeshBakeCoordinator() : worker(&NavmeshBakeCoordinator::Run, this) {}

NavmeshBakeCoordinator::~NavmeshBakeCoordinator() {
    {
        std::lock_guard lock(mutex);
        stopping = true;
        pending.reset();
    }
    wake.notify_one();
    if (worker.joinable()) worker.join();
}

std::uint64_t NavmeshBakeCoordinator::Request(const GeometryModel& model,
    const GeometryValidation& validation, double agent_radius, std::string input_key) {
    std::lock_guard lock(mutex);
    if (stopping) return next_generation;
    const auto generation = ++next_generation;
    pending = Job{generation, std::move(input_key), model, validation, agent_radius};
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
        ConstrainedDelaunay triangulator;
        auto result = triangulator.Generate(job.model, job.validation, job.agent_radius);
        {
            std::lock_guard lock(mutex);
            running = false;
            completion = BakeCompletion{job.generation, std::move(job.input_key), std::move(result)};
        }
    }
}

} // namespace cdt
