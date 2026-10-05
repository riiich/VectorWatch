#pragma once

#include <cmath>

namespace vectorwatch {

struct Vector3 {
    double x{};
    double y{};
    double z{};

    constexpr Vector3(
        double xValue = 0.0,
        double yValue = 0.0,
        double zValue = 0.0) noexcept
        : x(xValue), y(yValue), z(zValue) {}

    constexpr Vector3 operator+(const Vector3& other) const noexcept {
        return Vector3{x + other.x, y + other.y, z + other.z};
    }

    constexpr Vector3 operator-(const Vector3& other) const noexcept {
        return Vector3{x - other.x, y - other.y, z - other.z};
    }

    constexpr Vector3 operator*(double scalar) const noexcept {
        return Vector3{x * scalar, y * scalar, z * scalar};
    }

    constexpr double dot(const Vector3& other) const noexcept {
        return (x * other.x) + (y * other.y) + (z * other.z);
    }

    constexpr double lengthSquared() const noexcept {
        return dot(*this);
    }

    double length() const noexcept {
        return std::sqrt(lengthSquared());
    }
};

} // namespace vectorwatch
