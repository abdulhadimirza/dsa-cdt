#pragma once

#include "model/GeometryModel.hpp"
#include "validation/GeometryValidator.hpp"

#include <cstdint>
#include <string>

namespace cdt {

/// Keep the expensive validation report and its bake input signature ready for the current
/// geometry version and agent radius, so unchanged frames can reuse both results.
class GeometryDerivedDataCache {
public:
    /// Refresh the validation report if the model version changed, then refresh the signature
    /// if either that version or the supplied agent radius changed.
    void Refresh(const GeometryModel& model, const GeometryValidator& validator, double agent_radius);

    /// Return the report produced for the most recently refreshed geometry version.
    [[nodiscard]] const GeometryValidationReport& ValidationReport() const noexcept;
    /// Return the signature built from that report and the most recently supplied radius.
    [[nodiscard]] const std::string& BakeInputSignature() const noexcept;

private:
    struct ValidationReportCache {
        /// Replace the report only when it has never been built or the model version has advanced.
        void Refresh(const GeometryModel& model, const GeometryValidator& validator);

        /// Return the stored report for rendering and bake requests.
        [[nodiscard]] const GeometryValidationReport& Report() const noexcept { return report; }
        /// Return the model version from which the stored report was produced.
        [[nodiscard]] std::uint64_t GeometryVersion() const noexcept { return geometry_version; }

        /// Validation results reused until the model's geometry version changes.
        GeometryValidationReport report;
        /// Geometry version associated with report.
        std::uint64_t geometry_version{};
        /// Distinguishes an unbuilt cache from a valid report for version zero.
        bool initialized{false};
    };
    /// Keeps the report together with the geometry version it describes.
    ValidationReportCache validation;

    struct BakeInputSignatureCache {
        /// Rebuild the signature only when the report's geometry version or agent radius changes.
        void Refresh(const GeometryValidationReport& report, std::uint64_t geometry_version,
            double agent_radius);

        /// Return the stored signature used to compare or tag bake results.
        [[nodiscard]] const std::string& Signature() const noexcept { return signature; }

        /// Text value describing the validated geometry and radius used by a bake.
        std::string signature;
        /// Geometry version represented by signature.
        std::uint64_t geometry_version{};
        /// Agent radius represented by signature.
        double agent_radius{};
        /// Distinguishes an unbuilt signature from a valid empty-state signature.
        bool initialized{false};
    };
    /// Keeps the signature together with the geometry version and radius it describes.
    BakeInputSignatureCache bake_input;
};

} // namespace cdt
