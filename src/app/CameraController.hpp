#pragma once

#include "raylib.h"
#include "core/Vector2d.hpp"

namespace cdt {

class CameraController {
public:
    explicit CameraController(Vector2 initial_target = {0.0f, 0.0f});

    void Update();

    [[nodiscard]] bool IsPanning() const noexcept;
    [[nodiscard]] Vector2d ScreenToWorld(Vector2 screen_pos) const noexcept;
    [[nodiscard]] Vector2 WorldToScreen(Vector2d world_pos) const noexcept;

    [[nodiscard]] const Camera2D& GetCamera() const noexcept { return camera; }
    [[nodiscard]] Camera2D& GetCamera() noexcept { return camera; }

    [[nodiscard]] float GetZoom() const noexcept { return camera.zoom; }
    void SetZoom(float z) noexcept;

    [[nodiscard]] Vector2 GetTarget() const noexcept { return camera.target; }
    void ResetView(Vector2 target = {0.0f, 0.0f}) noexcept;

    void DrawWorldGrid() const;

private:
    Camera2D camera{};
    float min_zoom{0.05f};
    float max_zoom{25.0f};
};

} // namespace cdt
