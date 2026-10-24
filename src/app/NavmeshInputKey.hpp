#pragma once
#include "validation/GeometryValidator.hpp"
#include <string>

namespace cdt {
/// Stable bake identity for the validated geometry snapshot and agent radius.
[[nodiscard]] std::string MakeNavmeshInputKey(
    const std::optional<ValidatedNavmeshInput>& geometry, double agentRadius);
}
