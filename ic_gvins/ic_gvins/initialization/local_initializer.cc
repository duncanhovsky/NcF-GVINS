#include "initialization/local_initializer.h"

#include "common/rotation.h"
#include "misc.h"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <vector>

bool LocalInitializer::tryInitialize(
    const std::deque<std::pair<IMU, IntegrationState>> &ins_window,
    const HeadingObservation *heading, IntegrationState &state) const {
    const size_t requested =
        static_cast<size_t>(std::max(20.0, std::ceil(options_.static_duration * options_.imu_rate)));
    if (ins_window.size() < requested) {
        return false;
    }

    std::vector<IMU> imu_buffer;
    imu_buffer.reserve(requested);
    auto start = ins_window.end() - static_cast<std::ptrdiff_t>(requested);
    for (auto it = start; it != ins_window.end(); ++it) {
        imu_buffer.push_back(it->first);
    }

    std::vector<double> average;
    if (!MISC::detectZeroVelocity(imu_buffer, options_.imu_rate, average)) {
        return false;
    }

    const Vector3d gyro_bias =
        Vector3d(average[0], average[1], average[2]) * options_.imu_rate;
    const Vector3d fb =
        Vector3d(average[3], average[4], average[5]) * options_.imu_rate;
    Vector3d initial_attitude{0, 0, 0};
    initial_attitude[0] = -std::asin(fb[1] / options_.gravity);
    initial_attitude[1] = std::asin(fb[0] / options_.gravity);
    if (heading && heading->valid) {
        initial_attitude[2] = heading->yaw;
    }

    // NC-IC extension: local yaw is a zero gauge unless an explicitly enabled,
    // calibrated heading observation is available; raw magnetic data is never
    // consumed here.
    state.time = ins_window.back().first.time;
    state.p = Vector3d::Zero();
    state.q = Rotation::euler2quaternion(initial_attitude);
    state.v = Vector3d::Zero();
    state.bg = gyro_bias;
    state.ba = Vector3d::Zero();
    state.sodo = 0.0;
    state.sg = Vector3d::Zero();
    state.sa = Vector3d::Zero();
    return true;
}
