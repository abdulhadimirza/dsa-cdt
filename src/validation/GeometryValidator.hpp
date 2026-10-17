#pragma once

#include "model/GeometryModel.hpp"

#include <optional>
#include <vector>

namespace cdt {

struct LoopValidation {
    std::size_t loop_index{};
    bool individually_valid{false};
    bool active_border{false};
    bool eligible{false};
    bool inside_active_border{false};
    bool intersects_conflict{false};
};

struct GeometryValidation {
    std::optional<std::size_t> active_border_index;
    std::vector<LoopValidation> loops;
};

class GeometryValidator {
public:
    [[nodiscard]] GeometryValidation Validate(const GeometryModel& model) const;
};

} // namespace cdt
