#pragma once

template <typename R, typename T>
bool contains(R&& range, const T& value) {
    return std::find(std::begin(range), std::end(range), value) != std::end(range);
}