#include "core/GeometryPredicates.hpp"
#include "robust-predicates/geompred.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>

namespace cdt::geometry {
namespace {
// geompred's adaptive predicates require one-time floating-point environment setup; call_once
// keeps concurrent bakes safe while avoiding repeated initialization.
void EnsurePredicatesInitialized() {
    static std::once_flag initialized;
    std::call_once(initialized, [] { mp_float::exactinit(); });
}
}

// Fast arithmetic cross product for vector math; topological decisions use Orient2D below.
double Cross2D(Vector2d a, Vector2d b) noexcept { return a.x * b.y - a.y * b.x; }

// Adapt arguments to geompred's robust orientation predicate (positive is counterclockwise).
double Orient2D(Vector2d a, Vector2d b, Vector2d c) {
    EnsurePredicatesInitialized();
    const double pa[]{a.x, a.y}, pb[]{b.x, b.y}, pc[]{c.x, c.y};
    return geompred::orient2d(pa, pb, pc);
}

// The sign follows geompred's convention for a counterclockwise defining triangle.
double InCircle2D(Vector2d a, Vector2d b, Vector2d c, Vector2d d) {
    EnsurePredicatesInitialized();
    const double pa[]{a.x, a.y}, pb[]{b.x, b.y}, pc[]{c.x, c.y}, pd[]{d.x, d.y};
    return geompred::inball2d(pa, pb, pc, pd);
}

// Fan triangulation around the first point avoids subtracting large absolute shoelace terms.
double SignedTwiceArea(std::span<const Vector2d> polygon) noexcept {
    if (polygon.size() < 3) return 0.0;
    double area = 0.0;
    const Vector2d origin = polygon.front();
    for (std::size_t i = 1; i + 1 < polygon.size(); ++i)
        area += Cross2D(polygon[i] - origin, polygon[i + 1] - origin);
    return area;
}

// Ray crossing toggles interior state; exact edge membership is handled first according to policy.
bool PointInPolygon(std::span<const Vector2d> polygon, Vector2d point, BoundaryPolicy policy) {
    if (polygon.size() < 3) return false;
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const Vector2d a = polygon[i], b = polygon[j];
        const Vector2d edge = b - a, offset = point - a;
        if (Cross2D(edge, offset) == 0.0 &&
            point.x >= std::min(a.x, b.x) && point.x <= std::max(a.x, b.x) &&
            point.y >= std::min(a.y, b.y) && point.y <= std::max(a.y, b.y))
            return policy == BoundaryPolicy::Inside;
        if ((a.y > point.y) != (b.y > point.y) &&
            point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
}
}
