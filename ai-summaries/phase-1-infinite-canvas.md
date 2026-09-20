# Summary 1: Coordinate Decoupling, Zoom-Scaled Hit Testing, and Robust Interaction State Machine

**Date**: 2026-09-20  
**Context**: Interactive Infinite Canvas for Dynamic Constrained Delaunay Triangulation (`dsa-cdt`)  
**Target Audience**: Future AI Agents and Developers working on this codebase.

---

## 1. Project & System Overview

- **Project Vision**: Real-time Dynamic Constrained Delaunay Triangulation (DCDT) NavMesh and circular agent pathfinding simulation in modern C++23.
- **Key Libraries**: Raylib (rendering & windowing), Dear ImGui via `rlImGui` (HUD & controls).
- **Core Decoupling Rule**:
  - **Simulation & Geometry Layer**: Operates strictly in **World Space** using `cdt::Vector2d` (IEEE 754 double precision). Points, polygon vertices, obstacles, and triangle meshes exist in world units.
  - **Camera Layer**: `cdt::CameraController` wraps Raylib's `Camera2D`.
    - `camera.offset` = Screen center in physical pixels (`{ screenWidth / 2, screenHeight / 2 }`).
    - `camera.target` = World coordinate that is currently aligned with the screen center.
    - `camera.zoom` = Floating-point scale factor ($0.05\text{x}$ to $25.0\text{x}$).
    - **Cursor-Anchored Zoom**: Zooming recalculates `camera.target` so the world position underneath the cursor stays pinned under the mouse cursor.
    - **Canvas Pan**: Strictly bound to **`Space` + Left Mouse Drag** (Middle Mouse Drag was deliberately omitted to keep controls simple and conflict-free).
  - **Screen Space Layer**: Text labels, coordinates, bottom status bars, and ImGui windows are drawn outside `BeginMode2D(camera)` in raw pixel coordinates for crisp, unscaled rendering.

---

## 2. File Organization & Architecture

The codebase follows the modular design laid out in `ARCHITECTURE.md`:

```
dsa-cdt/
├── CMakeLists.txt
├── ARCHITECTURE.md
├── .vscode/
│   ├── c_cpp_properties.json      # IntelliSense configured via compile_commands.json
│   ├── launch.json                # GDB debugging configuration for VS Code (F5)
│   └── settings.json
├── src/
│   ├── core/
│   │   └── Vector2d.hpp           # Header-only double-precision 2D vector with inlined constexpr operators
│   ├── app/
│   │   ├── CameraController.hpp   # Clean class declaration & public interface
│   │   └── CameraController.cpp   # Camera update, pan/zoom math, infinite grid rendering
│   └── main.cpp                   # Interaction loop, world rendering & screen-space HUD/ImGui
```

### Key Architectural Decisions:
1. **`Vector2d.hpp` is Header-Only**:
   - Math primitives with `constexpr` / inline operator overloads are kept in the header so the compiler can inline vector operations directly into tight geometric loops (Lawson flips, orientation tests, nearest-neighbor searches).
2. **`CameraController` is Split (`.hpp` / `.cpp`)**:
   - Declarations live in `CameraController.hpp` and definitions in `CameraController.cpp`.
   - `src/app/CameraController.cpp` is explicitly added to `add_executable` in `CMakeLists.txt`.
3. **CMake Include Paths**:
   - `target_include_directories(${PROJECT_NAME} PRIVATE src ...)` is configured so files can cleanly use `#include "core/Vector2d.hpp"` and `#include "app/CameraController.hpp"`.

---

## 3. Infinite Grid & Coordinate Axes

`CameraController::DrawWorldGrid()` provides an unbounded visual reference:
1. **Dynamic Viewport Culling**: Computes visible world-space bounds from `GetScreenToWorld2D` at `(0, 0)` and `(screenWidth, screenHeight)`.
2. **Dual-Tier Spacing**:
   - **Minor Grid**: Spaced every 50 world units. Culled when zoomed out (`minor_spacing * camera.zoom >= 12.0f`) to prevent visual Moiré patterns and performance degradation.
   - **Major Grid**: Spaced every 250 world units. Always visible across normal zoom levels.
3. **World Origin Axes**:
   - X-Axis ($y = 0$): Highlighted with a subtle red accent line (`Color{230, 80, 80, 90}`).
   - Y-Axis ($x = 0$): Highlighted with a subtle green accent line (`Color{80, 200, 120, 90}`).

---

## 4. Problems Addressed & Solutions Implemented

### 4.1. Zoom-Scaled Hover Mechanism
- **The Bug**: Points were rendered inside `BeginMode2D` as circles with world-space radius $8.5$. Raylib scaled their visible size on screen by `camera.zoom` (e.g., radius was $85\text{px}$ at $10\text{x}$ zoom, or $1.7\text{px}$ at $0.2\text{x}$ zoom). However, hover detection tested screen pixels against a hardcoded constant `constexpr float pick_radius = 12.0f;`.
  - At high zoom: Hover only registered in a tiny inner circle near the center; touching the outer 80% of the visible circle failed to hover.
  - At low zoom: Hover triggered from $12\text{px}$ away in empty space, causing nearby points to conflict.
- **The Solution**:
  1. Extracted geometry constants into single sources of truth:
     ```cpp
     constexpr float point_outer_radius = 8.5f;   // World units
     constexpr float point_inner_radius = 5.5f;   // World units
     constexpr double base_pick_radius_world = point_outer_radius; // Exact visual match
     constexpr double min_pick_radius_screen = 8.0; // Accessibility floor when zoomed far out
     ```
  2. Performed hit testing directly in **World Space** against `mouse_world_pos` (`camera_controller.ScreenToWorld(mouse_screen_pos)`):
     $$\text{effective\_world\_radius} = \max\left(\text{base\_pick\_radius\_world}, \frac{\text{min\_pick\_radius\_screen}}{\text{zoom}}\right)$$
  3. This naturally scales the hover radius with zoom on screen while ensuring that when zoomed far out, the clickable area never drops below an $8\text{px}$ screen radius.

### 4.2. Fast Nearest-Point Tie-Breaking (Self Dot-Product)
- **The Bug**: The original search broke on the first point encountered (`break;`), ignoring closer points when clustered.
- **The Solution**: Replaced with nearest-neighbor search. To avoid calling `std::sqrt`, we use the vector displacement dot product with itself:
  ```cpp
  const cdt::Vector2d diff = mouse_world_pos - points[i].position;
  const double dist_sq = diff.dot(diff); // Equivalent to diff.length_sq(), avoids std::sqrt
  if (dist_sq <= min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_index = i;
  }
  ```

### 4.3. Removing Ghost Padding (`point_pick_margin`)
- An initial attempt included `point_pick_margin = 3.5f` padding. Testing revealed that this margin scaled with zoom ($3.5 \times 5 = 17.5\text{px}$ outside the circle at $5\text{x}$ zoom), making points turn yellow before the cursor touched the visible circle.
- Setting `base_pick_radius_world = point_outer_radius;` made hover hitboxes 100% pixel-perfect to the visible circle edge.

### 4.4. Offset-Preserving Dragging (Preventing Center-Snapping)
- **The Bug**: Clicking and dragging immediately snapped `points[dragged_index].position = mouse_world_pos;`, causing the circle to jump if clicked off-center.
- **The Solution**:
  - On mouse down: `drag_offset_world = points[hovered_point_index].position - mouse_world_pos;`
  - While dragging: `points[dragged_point_index].position = mouse_world_pos + drag_offset_world;`
  - Maintains exact cursor-to-center relative displacement throughout dragging.

### 4.5. Robust Drag State Machine & Right-Click Deletion (Out-of-Bounds Crash Fix)
- **The Bug**:
  1. If dragging with an offset or moving fast, cursor drift caused `hovered_point_index = -1`, failing deletion or deleting the wrong point passed under cursor.
  2. In debug builds, deleting the last point in the vector or clearing the vector left `hovered_point_index` pointing to a stale index (e.g. index 9 on size 9). ImGui then evaluated `points[hovered_point_index]`, triggering `std::__glibcxx_assert_fail` $\rightarrow$ `abort()` (Windows access violation / "Unknown signal").
- **The Solution**:
  1. Priority deletion with immediate index invalidation:
     ```cpp
     const int target_to_delete = (dragged_point_index != -1) ? dragged_point_index : hovered_point_index;
     if (target_to_delete != -1 && target_to_delete < static_cast<int>(points.size())) {
         points.erase(points.begin() + target_to_delete);
         dragged_point_index = -1;
         hovered_point_index = -1; // Critical: prevent stale out-of-bounds access in subsequent frames
     }
     ```
  2. Hover suppression: When `dragged_point_index != -1`, hover search is skipped and `hovered_point_index = dragged_point_index`, preventing points passed over from highlighting yellow.
  3. Panning & ImGui safety: If `mouse_captured || is_panning`, both `dragged_point_index` and `hovered_point_index` are reset to `-1`, safely dropping any drag before canvas panning begins.
  4. ImGui Button Resets: "Clear Points", "Reset Default Points", and "Generate Random Points" all reset `hovered_point_index = -1` and `dragged_point_index = -1`.
  5. ImGui Bounds Check:
     ```cpp
     if (hovered_point_index != -1 && hovered_point_index < static_cast<int>(points.size()))
     ```

### 4.6. Style Standardization on Sentinel `-1`
- Standardized all index presence and bounds checks on `!= -1` (instead of mixing `>= 0` and `!= -1`):
  ```cpp
  if (dragged_point_index != -1)
  if (hovered_point_index != -1)
  if (target_to_delete != -1 && target_to_delete < static_cast<int>(points.size()))
  if (hovered_point_index != -1 && hovered_point_index < static_cast<int>(points.size()))
  ```

---

## 5. Current Architecture of `src/main.cpp` Interaction Loop

```cpp
// 1. Coordinate & State Setup
const Vector2 mouse_screen_pos = GetMousePosition();
const bool mouse_captured = ImGui::GetIO().WantCaptureMouse;
if (!mouse_captured) camera_controller.Update();

const cdt::Vector2d mouse_world_pos = camera_controller.ScreenToWorld(mouse_screen_pos);
const bool is_panning = camera_controller.IsPanning() || IsKeyDown(KEY_SPACE);

// 2. Interaction State Machine
if (mouse_captured || is_panning) {
    hovered_point_index = -1;
    dragged_point_index = -1;
} else {
    // Clamp invalid drag index
    if (dragged_point_index >= static_cast<int>(points.size())) dragged_point_index = -1;

    if (dragged_point_index != -1) {
        // State A: Actively dragging
        hovered_point_index = dragged_point_index;
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            points[dragged_point_index].position = mouse_world_pos + drag_offset_world;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) dragged_point_index = -1;
    } else {
        // State B: Idle / Hovering
        // Nearest-point world search using diff.dot(diff)
        ...
        // Left-click: start drag or add new point
        ...
    }

    // Right-click: delete dragged or hovered point
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        const int target_to_delete = (dragged_point_index != -1) ? dragged_point_index : hovered_point_index;
        if (target_to_delete != -1 && target_to_delete < static_cast<int>(points.size())) {
            points.erase(points.begin() + target_to_delete);
            dragged_point_index = -1;
            hovered_point_index = -1;
        }
    }
}
```

---

## 6. Tooling, Environment & Debugging Knowledge

1. **IntelliSense Configuration (`.vscode/c_cpp_properties.json`)**:
   - **Do NOT** use `"includePath": ["${workspaceFolder}/**"]`. This causes VS Code to recursively parse all Raylib C99 internal files in `build/_deps/` as C++23, creating 200+ bogus errors in the Problems tab.
   - **Instead, use**:
     ```json
     "compileCommands": "${workspaceFolder}/build/compile_commands.json"
     ```
     This instructs IntelliSense to use the exact flags generated by CMake.
2. **GDB Debugging Setup (`.vscode/launch.json`)**:
   - Ready-to-run profile `"Debug RaylibImGuiApp (GDB)"` targets `${workspaceFolder}/build/RaylibImGuiApp.exe` with `gdb.exe`.
   - On Windows/MinGW, an Access Violation / SIGSEGV is often caught as an **"Unknown signal"**. In GDB:
     - Use Debug Console: `-exec bt` to view the call stack frames.
     - Frame #2/#3 will point directly to user code (e.g. `main.cpp`).
     - Check `-exec info locals` or `-exec print <var>` to inspect bounds.
3. **Build Execution Policy**:
   - The user preference is to **skip automatic terminal builds** unless explicitly approved or requested. Do not run `cmake --build` autonomously.

---

## 7. Key Takeaways for Future Tasks

1. **Always Check Coordinate Domain**: Never compare screen pixels with world coordinates directly. Use `camera_controller.ScreenToWorld()` or `camera_controller.WorldToScreen()`.
2. **Hit Testing World Entities**: Perform hit testing in **World Space** to avoid invoking Raylib matrix transforms for every entity each frame.
3. **Guard High & Low Zoom Extremes**: World-space hitboxes scale naturally, but remember to enforce a minimum screen floor (`min_screen_radius / zoom`) for when the camera is zoomed far out.
4. **Avoid Euclidean `sqrt` in Loops**: Use `diff.dot(diff)` against `radius * radius` for all distance and proximity tests.
5. **Preserve Drag Offsets**: When dragging an entity, store `offset = entity.position - mouse_position` at click time to prevent jarring center-snapping.
6. **Sentinel Consistency**: Treat `-1` as the uniform indicator for "no selection / no active entity".
7. **Pan Binding Convention**: Canvas panning is strictly `Space` + Left Mouse Drag.
8. **Keep Math Primitives Header-Only**: Foundation math structs (`Vector2d.hpp`) stay header-only with inlined `constexpr` operators; higher-level subsystems (`CameraController`, `CDT`, `AStar`) separate interface (`.hpp`) and implementation (`.cpp`).
