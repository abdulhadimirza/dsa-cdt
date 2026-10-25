#include "app/MapFile.hpp"

#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <utility>
#include <vector>

namespace cdt {
namespace {

bool HasTextExtension(const std::string& path) {
    std::string extension = std::filesystem::path(path).extension().string();
    for (char& character : extension)
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    return extension == ".txt";
}

} // namespace

bool SaveMapFile(const std::string& path, const GeometryModel& model, std::string& error) {
    if (!HasTextExtension(path)) {
        error = "Map file name must end in .txt.";
        return false;
    }
    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        error = "Could not open the map file for writing.";
        return false;
    }
    output.imbue(std::locale::classic());
    output << "DSA_CDT_MAP 1\nVERTICES " << model.Vertices().size() << '\n'
        << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const Vertex& vertex : model.Vertices())
        output << "V " << vertex.id << ' ' << vertex.position.x << ' ' << vertex.position.y << '\n';
    output << "EDGES " << model.Edges().size() << '\n';
    for (const GeometryEdge& edge : model.Edges())
        output << "E " << edge.first << ' ' << edge.second << ' '
            << (edge.type == EdgeType::Border ? "BORDER" : "OBSTACLE") << '\n';
    output.flush();
    if (!output) {
        error = "Could not write the complete map file.";
        return false;
    }
    error.clear();
    return true;
}

bool LoadMapFile(const std::string& path, GeometryModel& model, std::string& error) {
    if (!HasTextExtension(path)) {
        error = "Map file name must end in .txt.";
        return false;
    }
    std::ifstream input(path);
    if (!input) {
        error = "Could not open the map file for reading.";
        return false;
    }
    input.imbue(std::locale::classic());

    std::string marker;
    int version = 0;
    std::size_t vertex_count = 0;
    if (!(input >> marker >> version) || marker != "DSA_CDT_MAP" || version != 1 ||
        !(input >> marker >> vertex_count) || marker != "VERTICES") {
        error = "Map header is invalid or unsupported.";
        return false;
    }

    std::vector<Vertex> vertices;
    vertices.reserve(vertex_count);
    for (std::size_t i = 0; i < vertex_count; ++i) {
        Vertex vertex;
        if (!(input >> marker >> vertex.id >> vertex.position.x >> vertex.position.y) || marker != "V" ||
            !std::isfinite(vertex.position.x) || !std::isfinite(vertex.position.y)) {
            error = "Map contains an invalid vertex record.";
            return false;
        }
        vertices.push_back(vertex);
    }

    std::size_t edge_count = 0;
    if (!(input >> marker >> edge_count) || marker != "EDGES") {
        error = "Map edge section is missing or invalid.";
        return false;
    }
    std::vector<GeometryEdge> edges;
    edges.reserve(edge_count);
    for (std::size_t i = 0; i < edge_count; ++i) {
        GeometryEdge edge;
        std::string type;
        if (!(input >> marker >> edge.first >> edge.second >> type) || marker != "E") {
            error = "Map contains an invalid edge record.";
            return false;
        }
        if (type == "BORDER") edge.type = EdgeType::Border;
        else if (type == "OBSTACLE") edge.type = EdgeType::Obstacle;
        else {
            error = "Map contains an unknown edge type.";
            return false;
        }
        edges.push_back(edge);
    }
    input >> std::ws;
    if (!input.eof()) {
        error = "Map contains unexpected trailing data.";
        return false;
    }
    if (!model.Replace(std::move(vertices), std::move(edges))) {
        error = "Map contains duplicate or invalid geometry references.";
        return false;
    }
    error.clear();
    return true;
}

} // namespace cdt
