#pragma once

#include "model/GeometryModel.hpp"

#include <string>

namespace cdt {

/// Saves geometry to or loads geometry from the versioned plain-text map format.
[[nodiscard]] bool SaveMapFile(const std::string& path, const GeometryModel& model, std::string& error);
[[nodiscard]] bool LoadMapFile(const std::string& path, GeometryModel& model, std::string& error);

} // namespace cdt
