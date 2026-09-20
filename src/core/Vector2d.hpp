#pragma once

#include "raylib.h"
#include <cmath>

namespace cdt {

struct Vector2d {
    double x{0.0};
    double y{0.0};

    constexpr Vector2d() = default;
    constexpr Vector2d(double x_val, double y_val) : x(x_val), y(y_val) {}

    // Conversions with raylib's single-precision Vector2
    [[nodiscard]] constexpr Vector2 to_raylib() const noexcept {
        return Vector2{static_cast<float>(x), static_cast<float>(y)};
    }

    [[nodiscard]] static constexpr Vector2d from_raylib(Vector2 v) noexcept {
        return Vector2d{static_cast<double>(v.x), static_cast<double>(v.y)};
    }

    // Arithmetic operators
    [[nodiscard]] constexpr Vector2d operator+(const Vector2d& rhs) const noexcept {
        return Vector2d{x + rhs.x, y + rhs.y};
    }

    [[nodiscard]] constexpr Vector2d operator-(const Vector2d& rhs) const noexcept {
        return Vector2d{x - rhs.x, y - rhs.y};
    }

    [[nodiscard]] constexpr Vector2d operator*(double scalar) const noexcept {
        return Vector2d{x * scalar, y * scalar};
    }

    [[nodiscard]] constexpr Vector2d operator/(double scalar) const noexcept {
        return Vector2d{x / scalar, y / scalar};
    }

    [[nodiscard]] constexpr Vector2d operator-() const noexcept {
        return Vector2d{-x, -y};
    }

    constexpr Vector2d& operator+=(const Vector2d& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    constexpr Vector2d& operator-=(const Vector2d& rhs) noexcept {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    constexpr Vector2d& operator*=(double scalar) noexcept {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    constexpr Vector2d& operator/=(double scalar) noexcept {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    [[nodiscard]] constexpr bool operator==(const Vector2d& rhs) const noexcept {
        return x == rhs.x && y == rhs.y;
    }

    [[nodiscard]] constexpr bool operator!=(const Vector2d& rhs) const noexcept {
        return !(*this == rhs);
    }

    // Geometric utilities
    [[nodiscard]] constexpr double dot(const Vector2d& rhs) const noexcept {
        return x * rhs.x + y * rhs.y;
    }

    [[nodiscard]] constexpr double length_sq() const noexcept {
        return x * x + y * y;
    }

    [[nodiscard]] double length() const noexcept {
        return std::sqrt(length_sq());
    }

    [[nodiscard]] constexpr double distance_sq(const Vector2d& rhs) const noexcept {
        return (*this - rhs).length_sq();
    }

    [[nodiscard]] double distance(const Vector2d& rhs) const noexcept {
        return (*this - rhs).length();
    }
};

[[nodiscard]] constexpr Vector2d operator*(double scalar, const Vector2d& v) noexcept {
    return v * scalar;
}

} // namespace cdt
