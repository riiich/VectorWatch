#pragma once

#include "vectorwatch/Vector3.hpp"

namespace vectorwatch {

class Aircraft {
public:
    Aircraft(int id, Vector3 position, Vector3 velocity) noexcept
        : id_{id}, position_{position}, velocity_{velocity} {}

    [[nodiscard]] int id() const noexcept { return id_; }
    [[nodiscard]] const Vector3& position() const noexcept { return position_; }
    [[nodiscard]] const Vector3& velocity() const noexcept { return velocity_; }

    void update(double deltaTimeSeconds) noexcept {
        position_ = position_ + (velocity_ * deltaTimeSeconds);
    }

private:
    int id_;
    Vector3 position_;
    Vector3 velocity_;
};

} // namespace vectorwatch
