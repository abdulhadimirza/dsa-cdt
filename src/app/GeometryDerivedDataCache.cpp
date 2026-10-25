#include "app/GeometryDerivedDataCache.hpp"

#include "app/NavmeshInputSignature.hpp"

namespace cdt {

// Keep the previous report when the model still has the same version; otherwise validate the
// current model and remember which version produced the new report.
void GeometryDerivedDataCache::ValidationReportCache::Refresh(
    const GeometryModel& model, const GeometryValidator& validator) {
    if (initialized && geometry_version == model.Version()) return;

    report = validator.Validate(model);
    geometry_version = model.Version();
    initialized = true;
}

// Reuse the stored signature when both inputs still match; otherwise build it from the current
// report and radius, then remember which inputs produced it.
void GeometryDerivedDataCache::BakeInputSignatureCache::Refresh(
    const GeometryValidationReport& validation_report, std::uint64_t current_geometry_version,
    double current_agent_radius) {
    if (initialized && geometry_version == current_geometry_version && agent_radius == current_agent_radius)
        return;

    signature = MakeNavmeshInputSignature(validation_report.validated_navmesh_input, current_agent_radius);
    geometry_version = current_geometry_version;
    agent_radius = current_agent_radius;
    initialized = true;
}

// Refresh validation first so the signature is always made from the report for this model version.
void GeometryDerivedDataCache::Refresh(
    const GeometryModel& model, const GeometryValidator& validator, double agent_radius) {
    validation.Refresh(model, validator);
    bake_input.Refresh(validation.Report(), validation.GeometryVersion(), agent_radius);
}

// Expose the current report without copying its loop diagnostics or validated geometry.
const GeometryValidationReport& GeometryDerivedDataCache::ValidationReport() const noexcept {
    return validation.Report();
}

// Return the current signature without copying its text.
const std::string& GeometryDerivedDataCache::BakeInputSignature() const noexcept {
    return bake_input.Signature();
}

} // namespace cdt
