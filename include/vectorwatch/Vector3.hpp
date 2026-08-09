#pragma once

namespace vectorwatch {

struct Vector3 {
    double x{};
    double y{};
    double z{};

    [[nodiscard]] constexpr Vector3 operator+(const Vector3& other) const noexcept {
        return {x + other.x, y + other.y, z + other.z};
    }

    [[nodiscard]] constexpr Vector3 operator-(const Vector3& other) const noexcept {
        return {x - other.x, y - other.y, z - other.z};
    }

    [[nodiscard]] constexpr Vector3 operator*(double scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }

    [[nodiscard]] constexpr double dot(const Vector3& other) const noexcept {
        return (x * other.x) + (y * other.y) + (z * other.z);
    }

    [[nodiscard]] constexpr double lengthSquared() const noexcept {
        return dot(*this);
    }
};

} // namespace vectorwatch
