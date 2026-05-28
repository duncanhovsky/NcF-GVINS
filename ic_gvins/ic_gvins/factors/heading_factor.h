/*
 * NC-IC extension for IC-GVINS.
 *
 * The estimator consumes calibrated yaw observations, not raw magnetic field.
 * This factor is dormant by default because the selected datasets have no
 * usable magnetometer stream.
 */

#ifndef HEADING_FACTOR_H
#define HEADING_FACTOR_H

#include <ceres/ceres.h>

struct HeadingFactor {
    HeadingFactor(double yaw, double std)
        : yaw_(yaw)
        , std_(std) {
    }

    template <typename T>
    bool operator()(const T *const pose, T *residual) const {
        const T x = pose[3];
        const T y = pose[4];
        const T z = pose[5];
        const T w = pose[6];
        const T predicted_yaw = ceres::atan2(T(2.0) * (w * z + x * y),
                                             T(1.0) - T(2.0) * (y * y + z * z));
        const T difference = predicted_yaw - T(yaw_);
        residual[0] = ceres::atan2(ceres::sin(difference), ceres::cos(difference)) / T(std_);
        return true;
    }

    double yaw_;
    double std_;
};

#endif // HEADING_FACTOR_H
