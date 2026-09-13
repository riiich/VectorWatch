#pragma once

#include <new>
#include <type_traits>
#include <utility>

namespace vectorwatch {

// C++14 does not include std::optional. This small replacement stores a value
// directly inside itself and tracks whether that value currently exists.
template <typename T>
class Optional {
public:
    Optional() noexcept : containsValue_(false) {}

    Optional(const T& value) : containsValue_(false) {
        construct(value);
    }

    Optional(T&& value) : containsValue_(false) {
        construct(std::move(value));
    }

    Optional(const Optional& other) : containsValue_(false) {
        if (other.containsValue_) {
            construct(*other.pointer());
        }
    }

    Optional(Optional&& other) : containsValue_(false) {
        if (other.containsValue_) {
            construct(std::move(*other.pointer()));
        }
    }

    ~Optional() {
        reset();
    }

    Optional& operator=(const Optional& other) {
        if (this == &other) {
            return *this;
        }
        if (containsValue_ && other.containsValue_) {
            *pointer() = *other.pointer();
        } else if (other.containsValue_) {
            construct(*other.pointer());
        } else {
            reset();
        }
        return *this;
    }

    Optional& operator=(Optional&& other) {
        if (this == &other) {
            return *this;
        }
        if (containsValue_ && other.containsValue_) {
            *pointer() = std::move(*other.pointer());
        } else if (other.containsValue_) {
            construct(std::move(*other.pointer()));
        } else {
            reset();
        }
        return *this;
    }

    Optional& operator=(const T& value) {
        if (containsValue_) {
            *pointer() = value;
        } else {
            construct(value);
        }
        return *this;
    }

    Optional& operator=(T&& value) {
        if (containsValue_) {
            *pointer() = std::move(value);
        } else {
            construct(std::move(value));
        }
        return *this;
    }

    bool hasValue() const noexcept {
        return containsValue_;
    }

    explicit operator bool() const noexcept {
        return containsValue_;
    }

    T& operator*() {
        return *pointer();
    }

    const T& operator*() const {
        return *pointer();
    }

    T* operator->() {
        return pointer();
    }

    const T* operator->() const {
        return pointer();
    }

    T valueOr(const T& fallback) const {
        return containsValue_ ? *pointer() : fallback;
    }

    void reset() noexcept {
        if (containsValue_) {
            pointer()->~T();
            containsValue_ = false;
        }
    }

private:
    typedef typename std::aligned_storage<sizeof(T), alignof(T)>::type Storage;

    Storage storage_;
    bool containsValue_;

    T* pointer() {
        return reinterpret_cast<T*>(&storage_);
    }

    const T* pointer() const {
        return reinterpret_cast<const T*>(&storage_);
    }

    template <typename U>
    void construct(U&& value) {
        new (&storage_) T(std::forward<U>(value));
        containsValue_ = true;
    }
};

} // namespace vectorwatch
