#include "app/NavmeshInputKey.hpp"
#include <iomanip>
#include <sstream>

namespace cdt {

std::string MakeNavmeshInputKey(
    const std::optional<ValidatedNavmeshInput>& geometry, double agentRadius) {
    // Precision 17 round-trips doubles; loop order, source IDs, positions, and radius together
    // distinguish every validated bake input while preserving the no-border sentinel behavior.
    std::ostringstream key;
    key << std::setprecision(17) << 'R' << agentRadius << ';';
    if (!geometry) return "no-active-border";
    const auto appendLoop = [&](const ValidatedLoop& loop) {
        key << 'L' << loop.source_loop_index << ':';
        for (std::size_t i = 0; i < loop.vertex_ids.size(); ++i)
            key << loop.vertex_ids[i] << ',' << loop.vertices[i].x << ',' << loop.vertices[i].y << ';';
    };
    appendLoop(geometry->ActiveBorder());
    for (const auto& obstacle : geometry->EligibleObstacles()) appendLoop(obstacle);
    return key.str();
}
}
