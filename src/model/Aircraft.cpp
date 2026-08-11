#include "vectorwatch/model/Aircraft.hpp"

namespace vectorwatch {

Aircraft::Aircraft(int id, Vector3 position, Vector3 velocity) noexcept
    : id_{id}, position_{position}, velocity_{velocity} {}

int Aircraft::id() const noexcept {
    return id_;
}

const Vector3& Aircraft::position() const noexcept {
    return position_;
}

const Vector3& Aircraft::velocity() const noexcept {
    return velocity_;
}

void Aircraft::update(double deltaTimeSeconds) noexcept {
    position_ = position_ + (velocity_ * deltaTimeSeconds);
}

} // namespace vectorwatch
