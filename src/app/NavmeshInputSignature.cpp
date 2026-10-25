#include "app/NavmeshInputSignature.hpp"
#include <iomanip>
#include <sstream>

namespace cdt {

// Include the radius and every loop that affects baking. Write enough digits to preserve each
// number, and keep a fixed order so the same inputs always produce the same text.
std::string MakeNavmeshInputSignature(
    const std::optional<ValidatedNavmeshInput>& geometry, double agentRadius) {
    // Append each part to one text stream in the order described above.
    std::ostringstream signature;
    signature << std::setprecision(17) << 'R' << agentRadius << ';';
    // A missing active border has its own stable signature because no geometry can be baked yet.
    if (!geometry) return "no-active-border";
    // Append a loop's source index, vertex IDs, and coordinates in the validator's stable order.
    const auto appendLoop = [&](const ValidatedLoop& loop) {
        signature << 'L' << loop.source_loop_index << ':';
        for (std::size_t i = 0; i < loop.vertex_ids.size(); ++i)
            signature << loop.vertex_ids[i] << ',' << loop.vertices[i].x << ',' << loop.vertices[i].y << ';';
    };
    appendLoop(geometry->ActiveBorder());
    // Only eligible obstacles are part of the validated bake input.
    for (const auto& obstacle : geometry->EligibleObstacles()) appendLoop(obstacle);
    return signature.str();
}
}
