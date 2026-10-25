#pragma once
#include "validation/GeometryValidator.hpp"
#include <string>

namespace cdt {
/// Build a stable text signature for the validated border, eligible obstacles, and agent radius.
/// The same inputs always produce the same text, which lets the app identify matching bake results.
[[nodiscard]] std::string MakeNavmeshInputSignature(
    const std::optional<ValidatedNavmeshInput>& geometry, double agentRadius);
}
