#include "app/MapFile.hpp"

#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <utility>
#include <vector>

namespace cdt {
namespace {

using Json = nlohmann::json;

bool HasJsonExtension(const std::string& path) {
    std::string extension = std::filesystem::path(path).extension().string();
    for (char& character : extension)
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    return extension == ".json";
}

// Map IDs must be whole, nonnegative integers; accept either JSON integer storage form.
bool ReadId(const Json& value, VertexId& id) {
    if (!value.is_number_integer()) return false;
    if (value.is_number_unsigned()) {
        id = value.get<VertexId>();
        return true;
    }
    const auto signed_id = value.get<std::int64_t>();
    if (signed_id < 0) return false;
    id = static_cast<VertexId>(signed_id);
    return true;
}

} // namespace

// Standard file streams resolve relative paths from the process working directory for both save and load.
bool SaveMapFile(const std::string& path, const GeometryModel& model, std::string& error) {
    if (!HasJsonExtension(path)) {
        error = "Map file name must end in .json.";
        return false;
    }
    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        error = "Could not open the map file for writing.";
        return false;
    }
    // Version the format; vertices store IDs/positions and edges reference IDs with their border/obstacle type.
    Json document{
        {"format", "dsa-cdt-map"},
        {"version", 1},
        {"vertices", Json::array()},
        {"edges", Json::array()},
    };
    for (const Vertex& vertex : model.Vertices()) {
        document["vertices"].push_back({
            {"id", vertex.id}, {"x", vertex.position.x}, {"y", vertex.position.y},
        });
    }
    for (const GeometryEdge& edge : model.Edges()) {
        document["edges"].push_back({
            {"first", edge.first},
            {"second", edge.second},
            {"type", edge.type == EdgeType::Border ? "border" : "obstacle"},
        });
    }
    output << document.dump(2) << '\n';
    output.flush();
    if (!output) {
        error = "Could not write the complete map file.";
        return false;
    }
    error.clear();
    return true;
}

bool LoadMapFile(const std::string& path, GeometryModel& model, std::string& error) {
    if (!HasJsonExtension(path)) {
        error = "Map file name must end in .json.";
        return false;
    }
    std::ifstream input(path);
    if (!input) {
        error = "Could not open the map file for reading.";
        return false;
    }
    // Parse and validate temporary geometry before Replace so a bad file cannot partially change the map.
    Json document;
    try {
        document = Json::parse(input);
        if (!document.is_object() || document.value("format", std::string{}) != "dsa-cdt-map" ||
            !document.contains("version") || !document["version"].is_number_integer() ||
            document["version"].get<int>() != 1 || !document.contains("vertices") ||
            !document["vertices"].is_array() || !document.contains("edges") ||
            !document["edges"].is_array()) {
            error = "Map JSON has an invalid or unsupported format.";
            return false;
        }

        std::vector<Vertex> vertices;
        vertices.reserve(document["vertices"].size());
        for (const Json& item : document["vertices"]) {
            if (!item.is_object() || !item.contains("id") || !item.contains("x") || !item.contains("y") ||
                !item["x"].is_number() || !item["y"].is_number()) {
                error = "Map JSON contains an invalid vertex record.";
                return false;
            }
            Vertex vertex;
            if (!ReadId(item["id"], vertex.id)) {
                error = "Map JSON contains an invalid vertex ID.";
                return false;
            }
            vertex.position = {item["x"].get<double>(), item["y"].get<double>()};
            if (!std::isfinite(vertex.position.x) || !std::isfinite(vertex.position.y)) {
                error = "Map JSON contains a non-finite vertex position.";
                return false;
            }
            vertices.push_back(vertex);
        }

        std::vector<GeometryEdge> edges;
        edges.reserve(document["edges"].size());
        for (const Json& item : document["edges"]) {
            if (!item.is_object() || !item.contains("first") || !item.contains("second") ||
                !item.contains("type") || !item["type"].is_string()) {
                error = "Map JSON contains an invalid edge record.";
                return false;
            }
            GeometryEdge edge;
            if (!ReadId(item["first"], edge.first) || !ReadId(item["second"], edge.second)) {
                error = "Map JSON contains an invalid edge endpoint ID.";
                return false;
            }
            const std::string type = item["type"].get<std::string>();
            if (type == "border") edge.type = EdgeType::Border;
            else if (type == "obstacle") edge.type = EdgeType::Obstacle;
            else {
                error = "Map JSON contains an unknown edge type.";
                return false;
            }
            edges.push_back(edge);
        }
        if (!model.Replace(std::move(vertices), std::move(edges))) {
            error = "Map JSON contains duplicate or invalid geometry references.";
            return false;
        }
    } catch (const Json::exception& exception) {
        error = std::string("Could not parse map JSON: ") + exception.what();
        return false;
    }
    error.clear();
    return true;
}

} // namespace cdt
