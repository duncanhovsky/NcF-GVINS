#include "ROS/kaist_imu_frd_converter.h"

#include <array>
#include <cmath>
#include <iostream>

namespace {

bool near(double lhs, double rhs) {
    return std::abs(lhs - rhs) < 1.0e-12;
}

void expectNear(double lhs, double rhs, const char *name) {
    if (!near(lhs, rhs)) {
        std::cerr << name << " expected " << rhs << " got " << lhs << '\n';
        std::exit(1);
    }
}

} // namespace

int main() {
    const kaist_imu_frd::Vector3 flu{-0.32, 0.31, 9.78};
    const auto frd = kaist_imu_frd::fluToFrd(flu);

    expectNear(frd.x, -0.32, "x");
    expectNear(frd.y, -0.31, "y");
    expectNear(frd.z, -9.78, "z");

    const std::array<double, 9> covariance = {
        1.0, 2.0, 3.0,
        4.0, 5.0, 6.0,
        7.0, 8.0, 9.0,
    };
    const auto converted = kaist_imu_frd::fluToFrdCovariance(covariance);

    const std::array<double, 9> expected = {
        1.0, -2.0, -3.0,
        -4.0, 5.0, 6.0,
        -7.0, 8.0, 9.0,
    };

    for (size_t i = 0; i < converted.size(); ++i) {
        expectNear(converted[i], expected[i], "covariance");
    }

    return 0;
}
