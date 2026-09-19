# Dynamic Constrained Delaunay Triangulation (DCDT) & NavMesh Simulation
## Architectural Design Document (DSA Semester Project)

---

## 1. Project Vision & Scope

The objective is to build a real-time **Dynamic Constrained Delaunay Triangulation (DCDT)** navigation mesh and agent pathfinding simulation.

### Key Capabilities
1. **Infinite Canvas**: Unbounded 2D world with smooth panning, zooming, and coordinate decoupling.
2. **Interactive Polygons**: User can draw arbitrary closed polygonal obstacles or load predefined maps. Vertices are ignored until a polygon is closed.
3. **Dynamic DCDT**: Polygons can be dynamically translated, rotated, or edited in real time; the triangulation updates locally without recomputing unaffected regions.
4. **NavMesh Pathfinding**: The free-space triangulation acts as a navigation mesh. A dual-graph A* search identifies triangle corridors.
5. **Smooth Funnel Path & Circular NPC**: The Simple Fast Funnel Algorithm (SSFA / String Pulling) shortens corridor paths, adjusted by agent radius clearance so circular NPCs steer smoothly around corners without clipping obstacles.

---

## 2. Key Architectural Decisions

### Decision 1: Mesh Topology — Indexed Triangle-Neighbor Structure
*Selected over DCEL / Half-Edge and Pointer-Based Mesh Graphs*

#### Rationale
While Doubly-Connected Edge Lists (DCEL) are common for general planar subdivisions (arbitrary n-gons), triangulations possess a fixed topology: **every face has exactly 3 vertices and at most 3 adjacent neighbors**.

```
              v[0]
              /  \
             /    \
       n[2] /      \ n[1]
           /   T    \
          /          \
      v[1]------------v[2]
              n[0]
```
*Opposite convention: Neighbor `n[i]` shares the edge opposite to vertex `v[i]`.*

#### Data Layout
```cpp
struct Vertex {
    Vector2d position;           // World-space coordinates (double precision)
    int id{-1};
    bool is_constrained{false};  // Part of a polygon boundary
};

struct Triangle {
    std::array<int, 3> v{-1, -1, -1};             // Indices into vertices vector (CCW order)
    std::array<int, 3> n{-1, -1, -1};             // Indices of neighbor triangles (-1 = boundary)
    std::array<bool, 3> constrained{false, false, false}; // Is edge opposite to v[i] a constraint?
    bool is_obstacle{false};                      // True if triangle is inside a polygon obstacle
};

struct TriangulationMesh {
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
};
```

#### Why Not DCEL?
| Metric | DCEL / Half-Edge | Indexed Triangle-Neighbor |
| :--- | :--- | :--- |
| **Primary Unit** | Directed Half-Edge | Triangle (Face) |
| **Storage Overhead** | 5 fields $\times$ 2 per edge $\approx 30V$ pointers/integers | 3 vertices + 3 neighbors $\approx 12V$ integers (2.5x smaller) |
| **Memory Safety** | Pointers easily invalidated by `vector` resize | Array indices (`int`) remain completely valid |
| **Edge Flip** | Rewire 10–14 pointers across edges, faces, vertices | Swap 2 vertex indices, rewire 4 neighbor indices (~6 lines of code) |
| **Dual Graph (A\*)** | Hop through `face->halfedge->next->twin->face` | Immediate: `n[0], n[1], n[2]` are direct neighbor nodes |

---

### Decision 2: Geometric Robustness — Double Precision & Adaptive Predicates
*Selected over naive float determinants and heavy arbitrary-precision libraries (CGAL/GMP)*

#### Rationale
Delaunay triangulation relies on geometric tests (orientation and empty circumcircle). Standard floating-point arithmetic suffers from roundoff errors that cause:
- Inverted triangle winding orders.
- Infinite edge-flip loops when 4 points are nearly co-circular or 3 points are nearly collinear.

#### Solution: Shewchuk's Adaptive Robust Predicates
- Use standard IEEE 754 hardware `double` for all simulation positions.
- Drop in Jonathan Shewchuk's public-domain exact adaptive predicates (`predicates.h` / `predicates.c`):
  1. `orient2d(pa, pb, pc)`:
     - `> 0`: Point $C$ is strictly to the left of directed line $AB$ (Counter-Clockwise).
     - `< 0`: Point $C$ is strictly to the right of line $AB$ (Clockwise).
     - `== 0`: Points $A, B, C$ are collinear.
  2. `incircle(pa, pb, pc, pd)`:
     - `> 0`: Point $D$ lies strictly inside circumcircle of CCW triangle $ABC$ (Delaunay violation).
     - `< 0`: Point $D$ lies strictly outside circumcircle.
     - `== 0`: All 4 points are co-circular.
- **Zero Heavy Dependencies**: No GMP/CGAL linking issues; runs at native CPU speed 99.9% of the time, falling back to exact multi-precision filters only when edge cases demand it.

---

### Decision 3: Dynamic CDT Strategy — Incremental Lawson with Local Cavity Repair
*Selected over global full-frame rebuilds and complex kinetic data structures*

#### Algorithmic Workflow
1. **Point Insertion**:
   - Locate containing triangle using a directed triangle walk ($O(\sqrt{n})$ or $O(\log n)$ with stochastic jump).
   - Subdivide triangle into 3 new triangles.
   - Propagate Lawson edge flips on unconstrained edges until circumcircle condition holds.
2. **Constraint Enforcement (Segment $AB$)**:
   - Trace ray from $A$ to $B$ across the triangulation, collecting intersecting edges.
   - Remove intersecting edges, forming two pseudo-polygons on either side of $AB$.
   - Retriangulate pseudo-polygons, locking segment $AB$ as constrained.
3. **Dynamic Polygon Moving (DCDT)**:
   - Identify the local "star" / cavity of triangles around moving vertices.
   - Remove local constraints, restore local Delaunay property via reverse flips, move vertices, and re-insert constraints.
4. **Architectural Safeguard (Dual Engine Mode)**:
   - Expose a clean interface:
     ```cpp
     class CDT {
     public:
         void InsertPolygon(const Polygon& poly);
         void UpdatePolygonPosition(int polyId, Vector2d delta);
         void RebuildAll(); // Full rebuild fallback for verification & benchmarking
     };
     ```
   - This allows early development of pathfinding using full rebuilds, followed by drop-in replacement with local dynamic repair without touching renderer or NPC systems.

---

### Decision 4: Canvas Architecture — Decoupled World vs. Screen Space
*Selected over screen-space geometry*

#### Coordinate Pipeline
- **Simulation Layer**: Operates exclusively in mathematical **World Space** coordinates (`Vector2d`).
- **Camera Layer**: Raylib `Camera2D` maintains:
  - `target` (world coordinate at screen center)
  - `offset` (screen center in pixels)
  - `zoom` (scale factor, e.g. 0.1x to 10.0x)
  - `rotation` (0.0)
- **Input Rule**: All mouse inputs immediately undergo `GetScreenToWorld2D(GetMousePosition(), camera)` before reaching polygon/mesh logic.
- **Rendering Rule**:
  ```cpp
  BeginDrawing();
      ClearBackground(DARK_BG);
      
      BeginMode2D(camera);
          // Render World Grid, Mesh, Constraints, NPC, Paths
      EndMode2D();
      
      // Render Screen-space HUD & ImGui Windows
      rlImGuiBegin();
      // ... ImGui controls ...
      rlImGuiEnd();
  EndDrawing();
  ```

---

### Decision 5: Pathfinding & Smooth Steering Pipeline
*Selected over grid-based A\* or center-point waypoint graphs*

```
[Start & Target Points]
         │
         ▼
[Point-In-Triangle Location]
         │
         ▼
[Dual Graph A* Search]  ──► Produces corridor of traversable triangles
         │
         ▼
[Portal Edge Extraction] ──► Extracts shared edges [Left_i, Right_i]
         │
         ▼
[Radius Clearance Shrink]──► Inward retraction of portal endpoints by R_npc
         │
         ▼
[Simple Fast Funnel (SSFA)]──► Taut string pulling generates smooth polygonal path
         │
         ▼
[NPC Kinematic Steering] ──► Smooth waypoint tracking with corner anticipation
```

1. **Dual Graph A\***: Nodes are traversable triangles (`is_obstacle == false`). Graph edges are shared unconstrained edges (`n[i] != -1 && !constrained[i]`).
2. **Portals**: Each transition between triangle $A$ and triangle $B$ yields a directed line segment $[L, R]$.
3. **Agent Radius Clearance**:
   - For an agent of radius $R_{npc}$, retract endpoint $L$ towards $R$ by $R_{npc}$, and $R$ towards $L$ by $R_{npc}$.
   - If portal width $< 2 R_{npc}$, edge is flagged impassable during A*.
4. **Funnel Algorithm**: Maintains apex, left ray, and right ray, advancing through portal vertices to compute the true shortest path in $O(k)$ time where $k$ is corridor length.

---

## 3. Modular Codebase Architecture

```
dsa-cdt/
├── CMakeLists.txt
├── ARCHITECTURE.md                 # This document
├── src/
│   ├── core/                       # Foundation math & exact predicates
│   │   ├── Vector2d.hpp            # Double-precision 2D vector
│   │   ├── Predicates.hpp          # Shewchuk's Orient2D & InCircle wrappers
│   │   └── predicates.c            # Jonathan Shewchuk's exact arithmetic kernel
│   │
│   ├── geometry/                   # High-level geometric abstractions
│   │   ├── Segment.hpp             # 2D segments, intersections, bounding boxes
│   │   └── Polygon.hpp             # Closed polygon definition & point-in-poly
│   │
│   ├── mesh/                       # Core Triangulation Data Structure
│   │   ├── MeshTypes.hpp           # Vertex, Triangle, Edge definitions
│   │   ├── LawsonFlips.hpp         # O(1) in-place topological edge flips
│   │   └── CDT.hpp / CDT.cpp       # Constrained Delaunay engine & dynamic repair
│   │
│   ├── pathfinding/                # NavMesh graph & path generation
│   │   ├── DualGraph.hpp           # Adjacency view for A*
│   │   ├── AStar.hpp / AStar.cpp   # Triangle corridor search
│   │   └── Funnel.hpp / Funnel.cpp # Simple Fast Funnel Algorithm (SSFA) with clearance
│   │
│   ├── sim/                        # Agent simulation
│   │   └── NPC.hpp / NPC.cpp       # Circular agent, kinematics, path follower
│   │
│   ├── app/                        # User interaction & canvas
│   │   ├── CameraController.hpp    # Pan/zoom controller
│   │   ├── PolygonDraftTool.hpp    # State machine for drawing polygons
│   │   └── Renderer.hpp            # Raylib visualizer (mesh, portals, funnel)
│   │
│   └── main.cpp                    # Application entry & ImGui coordinator
```

---

## 4. Phase-by-Phase Implementation Roadmap

- [x] **Phase 0: Environment & Modern C++ Setup** (CMake, C++23, Raylib, Dear ImGui).
- [ ] **Phase 1: Infinite Canvas & Polygon Drafting Tool**
  - Implement `CameraController` (`Camera2D`, mouse pan/zoom, screen-to-world).
  - Implement `PolygonDraftTool` (collect points, preview dashed lines, commit on loop closure).
- [ ] **Phase 2: Core Math & Predicates Kernel**
  - Integrate Shewchuk's `predicates.c` with clean `Vector2d` C++ interface.
  - Unit tests for `Orient2D` and `InCircle`.
- [ ] **Phase 3: Static Constrained Delaunay Triangulation (CDT)**
  - Indexed Triangle-Neighbor mesh structure.
  - Lawson incremental insertion + edge flips.
  - Polygon constraint insertion + obstacle interior marking.
- [ ] **Phase 4: Triangle Corridor A\* & Funnel Algorithm**
  - Dual graph traversal.
  - Simple Fast Funnel Algorithm implementation.
  - Agent radius $R_{npc}$ portal shrinking.
- [ ] **Phase 5: Circular NPC Simulation & Real-Time Dynamic DCDT**
  - Interactive polygon translation / dragging with mouse.
  - Dynamic local CDT cavity retriangulation.
  - Dynamic real-time path regeneration as obstacles move.
