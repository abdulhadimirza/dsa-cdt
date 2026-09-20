#include "app/CameraController.hpp"

#include "raymath.h"
#include <algorithm>
#include <cmath>

namespace cdt {

CameraController::CameraController(Vector2 initial_target) {
    camera.target = initial_target;
    camera.offset = {static_cast<float>(GetScreenWidth()) * 0.5f, static_cast<float>(GetScreenHeight()) * 0.5f};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
}

void CameraController::Update() {
    // Keep offset centered if the window was resized
    if (IsWindowResized()) {
        camera.offset = {static_cast<float>(GetScreenWidth()) * 0.5f, static_cast<float>(GetScreenHeight()) * 0.5f};
    }

    // 1. Pan Handling (Space + Left Mouse Drag)
    if (IsPanning()) {
        const Vector2 delta = GetMouseDelta();
        camera.target.x -= delta.x / camera.zoom;
        camera.target.y -= delta.y / camera.zoom;
    }

    // 2. Zoom Handling (Anchored to Mouse Cursor)
    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
        const Vector2 mouse_screen_pos = GetMousePosition();
        const Vector2 mouse_world_before = GetScreenToWorld2D(mouse_screen_pos, camera);

        constexpr float zoom_step = 1.15f;
        float new_zoom = (wheel > 0.0f) ? (camera.zoom * zoom_step) : (camera.zoom / zoom_step);
        new_zoom = std::clamp(new_zoom, min_zoom, max_zoom);

        camera.zoom = new_zoom;

        const Vector2 mouse_world_after = GetScreenToWorld2D(mouse_screen_pos, camera);
        camera.target.x += (mouse_world_before.x - mouse_world_after.x);
        camera.target.y += (mouse_world_before.y - mouse_world_after.y);
    }
}

bool CameraController::IsPanning() const noexcept {
    return IsKeyDown(KEY_SPACE) && IsMouseButtonDown(MOUSE_BUTTON_LEFT);
}

Vector2d CameraController::ScreenToWorld(Vector2 screen_pos) const noexcept {
    return Vector2d::from_raylib(GetScreenToWorld2D(screen_pos, camera));
}

Vector2 CameraController::WorldToScreen(Vector2d world_pos) const noexcept {
    return GetWorldToScreen2D(world_pos.to_raylib(), camera);
}

void CameraController::SetZoom(float z) noexcept {
    camera.zoom = std::clamp(z, min_zoom, max_zoom);
}

void CameraController::ResetView(Vector2 target) noexcept {
    camera.target = target;
    camera.offset = {static_cast<float>(GetScreenWidth()) * 0.5f, static_cast<float>(GetScreenHeight()) * 0.5f};
    camera.zoom = 1.0f;
    camera.rotation = 0.0f;
}

void CameraController::DrawWorldGrid() const {
    const Vector2 top_left = GetScreenToWorld2D(Vector2{0.0f, 0.0f}, camera);
    const Vector2 bottom_right = GetScreenToWorld2D(
        Vector2{static_cast<float>(GetScreenWidth()), static_cast<float>(GetScreenHeight())},
        camera
    );

    const float min_x = std::min(top_left.x, bottom_right.x);
    const float max_x = std::max(top_left.x, bottom_right.x);
    const float min_y = std::min(top_left.y, bottom_right.y);
    const float max_y = std::max(top_left.y, bottom_right.y);

    constexpr float minor_spacing = 50.0f;
    constexpr float major_spacing = 250.0f;

    // Draw minor grid lines only if they are spread apart enough on screen
    if (minor_spacing * camera.zoom >= 12.0f) {
        const Color minor_color{255, 255, 255, 10};

        const int start_x = static_cast<int>(std::floor(min_x / minor_spacing));
        const int end_x   = static_cast<int>(std::ceil(max_x / minor_spacing));
        for (int i = start_x; i <= end_x; ++i) {
            const float x = static_cast<float>(i) * minor_spacing;
            DrawLineV(Vector2{x, min_y}, Vector2{x, max_y}, minor_color);
        }

        const int start_y = static_cast<int>(std::floor(min_y / minor_spacing));
        const int end_y   = static_cast<int>(std::ceil(max_y / minor_spacing));
        for (int j = start_y; j <= end_y; ++j) {
            const float y = static_cast<float>(j) * minor_spacing;
            DrawLineV(Vector2{min_x, y}, Vector2{max_x, y}, minor_color);
        }
    }

    // Draw major grid lines
    {
        const Color major_color{255, 255, 255, 24};

        const int start_x = static_cast<int>(std::floor(min_x / major_spacing));
        const int end_x   = static_cast<int>(std::ceil(max_x / major_spacing));
        for (int i = start_x; i <= end_x; ++i) {
            const float x = static_cast<float>(i) * major_spacing;
            DrawLineV(Vector2{x, min_y}, Vector2{x, max_y}, major_color);
        }

        const int start_y = static_cast<int>(std::floor(min_y / major_spacing));
        const int end_y   = static_cast<int>(std::ceil(max_y / major_spacing));
        for (int j = start_y; j <= end_y; ++j) {
            const float y = static_cast<float>(j) * major_spacing;
            DrawLineV(Vector2{min_x, y}, Vector2{max_x, y}, major_color);
        }
    }

    // Draw World Coordinate Axes (X = Red tint, Y = Green tint)
    constexpr Color x_axis_color{230, 80, 80, 90};
    constexpr Color y_axis_color{80, 200, 120, 90};

    if (min_x <= 0.0f && max_x >= 0.0f) {
        DrawLineV(Vector2{0.0f, min_y}, Vector2{0.0f, max_y}, y_axis_color);
    }
    if (min_y <= 0.0f && max_y >= 0.0f) {
        DrawLineV(Vector2{min_x, 0.0f}, Vector2{max_x, 0.0f}, x_axis_color);
    }
}

} // namespace cdt
