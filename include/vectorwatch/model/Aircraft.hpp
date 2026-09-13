#pragma once

#include "vectorwatch/math/Vector3.hpp"

namespace vectorwatch {

class Aircraft {
public:
    Aircraft(int id, Vector3 position, Vector3 velocity) noexcept;

    int id() const noexcept;
    const Vector3& position() const noexcept;
    const Vector3& velocity() const noexcept;

    void update(double deltaTimeSeconds) noexcept;

private:
    int id_;
    Vector3 position_;
    Vector3 velocity_;
};

} // namespace vectorwatch
