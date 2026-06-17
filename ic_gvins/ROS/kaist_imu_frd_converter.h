#pragma once

#include <array>
#include <cstddef>

namespace kaist_imu_frd {

struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

inline Vector3 fluToFrd(const Vector3 &value) {
    return {value.x, -value.y, -value.z};
}

inline std::array<double, 9> fluToFrdCovariance(const std::array<double, 9> &covariance) {
    constexpr std::array<double, 3> signs{{1.0, -1.0, -1.0}};

    std::array<double, 9> converted{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
            converted[row * 3 + col] = signs[row] * covariance[row * 3 + col] * signs[col];
        }
    }
    return converted;
}

} // namespace kaist_imu_frd
