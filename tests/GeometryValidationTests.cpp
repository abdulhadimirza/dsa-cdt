#include "model/GeometryModel.hpp"
#include "validation/GeometryValidator.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

void Check(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}

std::vector<cdt::VertexId> AddLoop(cdt::GeometryModel& model,
    const std::vector<cdt::Vector2d>& points, cdt::EdgeType type) {
    std::vector<cdt::VertexId> ids;
    for (const auto point : points) ids.push_back(model.AddVertex(point));
    for (std::size_t i = 0; i < ids.size(); ++i)
        Check(model.AddEdge(ids[i], ids[(i + 1) % ids.size()], type), "loop edge added");
    return ids;
}

cdt::GeometryValidation Validate(const cdt::GeometryModel& model) {
    return cdt::GeometryValidator{}.Validate(model);
}

void TestLoopValidity() {
    cdt::GeometryModel model;
    AddLoop(model, {{0, 0}, {10, 0}, {10, 10}, {0, 10}}, cdt::EdgeType::Border);
    AddLoop(model, {{2, 2}, {8, 8}, {2, 8}, {8, 2}}, cdt::EdgeType::Obstacle);
    Check(model.Loops().size() == 2, "closed loops discovered");
    const auto validation = Validate(model);
    Check(validation.loops[0].individually_valid, "simple border valid");
    Check(!validation.loops[1].individually_valid, "bow-tie obstacle invalid");
    double twice_area = 0.0;
    for (std::size_t i = 0; i < model.Loops()[0].vertices.size(); ++i) {
        const auto* a = model.FindVertex(model.Loops()[0].vertices[i]);
        const auto* b = model.FindVertex(model.Loops()[0].vertices[
            (i + 1) % model.Loops()[0].vertices.size()]);
        twice_area += a->position.x * b->position.y - b->position.x * a->position.y;
    }
    Check(twice_area > 0.0, "loop vertices remain counterclockwise");

    cdt::GeometryModel open;
    const auto a = open.AddVertex({0, 0});
    const auto b = open.AddVertex({1, 0});
    Check(open.AddEdge(a, b, cdt::EdgeType::Border), "open edge added");
    Check(open.Loops().empty(), "open chain is not a loop");
}

void TestAdjacentEdgeValidity() {
    cdt::GeometryModel model;
    AddLoop(model, {{0, 0}, {4, 0}, {2, 0}, {2, 3}, {0, 3}}, cdt::EdgeType::Obstacle);
    AddLoop(model, {{10, 0}, {14, 0}, {14, 0}, {14, 3}, {10, 3}}, cdt::EdgeType::Obstacle);
    AddLoop(model, {{20, 0}, {22, 0}, {24, 0}, {24, 3}, {20, 3}}, cdt::EdgeType::Obstacle);
    AddLoop(model, {{30, 0}, {34, 0}, {34, 3}, {30, 3}}, cdt::EdgeType::Obstacle);
    const auto validation = Validate(model);
    Check(!validation.loops[0].individually_valid, "adjacent reverse edges are invalid");
    Check(!validation.loops[1].individually_valid, "zero-length edge is invalid");
    Check(validation.loops[2].individually_valid, "collinear edges continuing forward are valid");
    Check(validation.loops[3].individually_valid, "ordinary adjacent corner is valid");
}

void TestScaleAwareValidity() {
    for (const double scale : {1e-6, 1.0, 1e6}) {
        cdt::GeometryModel model;
        AddLoop(model, {{0, 0}, {4 * scale, 0}, {4 * scale, 3 * scale}, {0, 3 * scale}},
            cdt::EdgeType::Border);
        AddLoop(model, {{scale, scale}, {2 * scale, scale}, {2 * scale, 2 * scale}, {scale, 2 * scale}},
            cdt::EdgeType::Obstacle);
        const auto validation = Validate(model);
        Check(validation.loops[0].individually_valid, "border valid at coordinate scale");
        Check(validation.loops[1].individually_valid && validation.loops[1].eligible,
            "contained obstacle valid at coordinate scale");
    }
}

void TestBorderOrderAndFallback() {
    cdt::GeometryModel model;
    const auto first = AddLoop(model, {{0, 0}, {10, 0}, {10, 10}, {0, 10}}, cdt::EdgeType::Border);
    AddLoop(model, {{20, 0}, {30, 0}, {30, 10}, {20, 10}}, cdt::EdgeType::Border);
    auto validation = Validate(model);
    Check(validation.active_border_index.has_value(), "active border chosen");
    Check(model.Loops()[*validation.active_border_index].completion_order ==
        model.Loops()[0].completion_order, "first completed border selected");

    Check(model.MoveVertex(first[2], {0, 0}), "border vertex moved");
    validation = Validate(model);
    Check(validation.active_border_index.has_value(), "fallback border selected");
    Check(model.Loops()[*validation.active_border_index].completion_order >
        model.Loops()[0].completion_order, "later valid border becomes active");
}

void TestObstacleContainmentAndBorderPriority() {
    cdt::GeometryModel model;
    AddLoop(model, {{0, 0}, {20, 0}, {20, 20}, {0, 20}}, cdt::EdgeType::Border);
    AddLoop(model, {{4, 4}, {8, 4}, {8, 8}, {4, 8}}, cdt::EdgeType::Obstacle);
    AddLoop(model, {{22, 2}, {26, 2}, {26, 6}, {22, 6}}, cdt::EdgeType::Obstacle);
    AddLoop(model, {{0, 10}, {4, 9}, {4, 11}}, cdt::EdgeType::Obstacle);
    const auto validation = Validate(model);
    Check(validation.active_border_index.has_value(), "border remains active with obstacle contacts");
    Check(validation.loops[1].eligible, "contained obstacle eligible");
    Check(!validation.loops[2].eligible, "outside obstacle ineligible");
    Check(!validation.loops[3].eligible, "border-touching obstacle ineligible");
    Check(validation.loops[*validation.active_border_index].individually_valid,
        "obstacle contact does not invalidate border");
}

void TestObstacleConflicts() {
    cdt::GeometryModel model;
    AddLoop(model, {{0, 0}, {30, 0}, {30, 30}, {0, 30}}, cdt::EdgeType::Border);
    AddLoop(model, {{4, 4}, {12, 4}, {12, 12}, {4, 12}}, cdt::EdgeType::Obstacle);
    AddLoop(model, {{10, 8}, {18, 8}, {18, 16}, {10, 16}}, cdt::EdgeType::Obstacle);
    AddLoop(model, {{20, 20}, {24, 20}, {24, 24}, {20, 24}}, cdt::EdgeType::Obstacle);
    const auto validation = Validate(model);
    Check(!validation.loops[1].eligible && !validation.loops[2].eligible,
        "overlapping obstacles both ineligible");
    Check(validation.loops[1].intersects_conflict && validation.loops[2].intersects_conflict,
        "overlap reported on both loops");
    Check(validation.loops[3].eligible, "separate obstacle remains eligible");
}

void TestOutOfBorderObstaclesAreIgnoredForConflicts() {
    cdt::GeometryModel model;
    AddLoop(model, {{0, 0}, {20, 0}, {20, 20}, {0, 20}}, cdt::EdgeType::Border);
    AddLoop(model, {{16, 5}, {19, 5}, {19, 9}, {16, 9}}, cdt::EdgeType::Obstacle);
    AddLoop(model, {{18, 5}, {22, 5}, {22, 9}, {18, 9}}, cdt::EdgeType::Obstacle);
    AddLoop(model, {{22, 5}, {26, 5}, {26, 9}, {22, 9}}, cdt::EdgeType::Obstacle);
    AddLoop(model, {{20, 12}, {24, 12}, {24, 16}, {20, 16}}, cdt::EdgeType::Obstacle);
    const auto validation = Validate(model);
    Check(validation.loops[1].inside_active_border && validation.loops[1].eligible,
        "partly outside obstacle cannot disqualify fully inside obstacle");
    Check(!validation.loops[1].intersects_conflict, "ignored outside obstacles do not report conflicts");
    Check(!validation.loops[2].inside_active_border && !validation.loops[2].eligible,
        "partly outside obstacle remains ineligible");
    Check(!validation.loops[3].inside_active_border && !validation.loops[3].eligible,
        "fully outside obstacle remains ineligible");
    Check(!validation.loops[4].inside_active_border && !validation.loops[4].eligible,
        "border-touching obstacle remains ineligible");
}

void TestPointAndSegmentContactConflicts() {
    cdt::GeometryModel point_contact;
    AddLoop(point_contact, {{0, 0}, {30, 0}, {30, 30}, {0, 30}}, cdt::EdgeType::Border);
    AddLoop(point_contact, {{5, 5}, {12, 5}, {8, 12}}, cdt::EdgeType::Obstacle);
    AddLoop(point_contact, {{8, 12}, {15, 18}, {5, 18}}, cdt::EdgeType::Obstacle);
    auto validation = Validate(point_contact);
    Check(!validation.loops[1].eligible && !validation.loops[2].eligible,
        "point-touching obstacles conflict");

    cdt::GeometryModel segment_contact;
    AddLoop(segment_contact, {{0, 0}, {30, 0}, {30, 30}, {0, 30}}, cdt::EdgeType::Border);
    AddLoop(segment_contact, {{4, 4}, {10, 4}, {10, 12}, {4, 12}}, cdt::EdgeType::Obstacle);
    AddLoop(segment_contact, {{10, 4}, {16, 4}, {16, 12}, {10, 12}}, cdt::EdgeType::Obstacle);
    validation = Validate(segment_contact);
    Check(!validation.loops[1].eligible && !validation.loops[2].eligible,
        "shared-segment obstacles conflict");
}

void TestRevalidationAfterDeletion() {
    cdt::GeometryModel model;
    const auto first = AddLoop(model, {{0, 0}, {10, 0}, {10, 10}, {0, 10}}, cdt::EdgeType::Border);
    AddLoop(model, {{20, 0}, {30, 0}, {30, 10}, {20, 10}}, cdt::EdgeType::Border);
    for (const auto id : first) Check(model.RemoveVertex(id), "border vertex removed");
    const auto validation = Validate(model);
    Check(validation.active_border_index.has_value(), "remaining border selected after deletion");
    Check(model.Loops()[*validation.active_border_index].completion_order ==
        model.Loops().front().completion_order, "remaining loop keeps completion identity");
}

} // namespace

int main() {
    TestLoopValidity();
    TestAdjacentEdgeValidity();
    TestScaleAwareValidity();
    TestBorderOrderAndFallback();
    TestObstacleContainmentAndBorderPriority();
    TestObstacleConflicts();
    TestOutOfBorderObstaclesAreIgnoredForConflicts();
    TestPointAndSegmentContactConflicts();
    TestRevalidationAfterDeletion();
    std::cout << "Geometry validation tests passed\n";
}
