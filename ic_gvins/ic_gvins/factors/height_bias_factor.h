/*
 * NC-IC extension for IC-GVINS.
 *
 * These factors separate slow GNSS Down-axis measurement bias from the
 * odom/map recovery deviation.  Short height outliers remain the job of the
 * vertical health gate and must not be absorbed into this random walk.
 */

#ifndef HEIGHT_BIAS_FACTOR_H
#define HEIGHT_BIAS_FACTOR_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <algorithm>
#include <cmath>

#include "common/types.h"
#include "common/rotation.h"

class HeightBiasGnssFactor : public ceres::SizedCostFunction<3, 7, 1> {

public:
    explicit HeightBiasGnssFactor(GNSS gnss, Vector3d lever)
        : gnss_(std::move(gnss))
        , lever_(std::move(lever)) {
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const Vector3d p{parameters[0][0], parameters[0][1], parameters[0][2]};
        const Quaterniond q{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};
        Vector3d measurement = gnss_.blh;
        if (gnss_.use_online_offset && gnss_.recovery_deviation.valid) {
            const Matrix3d yaw_rotation =
                Eigen::AngleAxisd(gnss_.recovery_deviation.yaw, Vector3d::UnitZ()).toRotationMatrix();
            measurement = yaw_rotation * gnss_.raw_local + gnss_.recovery_deviation.translation;
        }
        measurement[2] += parameters[1][0];

        Eigen::Map<Eigen::Matrix<double, 3, 1>> error(residuals);
        error = p + q.toRotationMatrix() * lever_ - measurement;
        Matrix3d sqrt_info = Matrix3d::Zero();
        if (gnss_.horizontal_valid) {
            sqrt_info(0, 0) = 1.0 / gnss_.std[0];
            sqrt_info(1, 1) = 1.0 / gnss_.std[1];
        }
        if (gnss_.vertical_valid) {
            sqrt_info(2, 2) = 1.0 / gnss_.std[2];
        }
        error = sqrt_info * error;

        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
                jacobian_pose.setZero();
                jacobian_pose.block<3, 3>(0, 0) = Matrix3d::Identity();
                jacobian_pose.block<3, 3>(0, 3) =
                    -q.toRotationMatrix() * Rotation::skewSymmetric(lever_);
                jacobian_pose = sqrt_info * jacobian_pose;
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 3, 1>> jacobian_bias(jacobians[1]);
                jacobian_bias.setZero();
                jacobian_bias[2] = gnss_.vertical_valid ? -1.0 / gnss_.std[2] : 0.0;
            }
        }
        return true;
    }

private:
    GNSS gnss_;
    Vector3d lever_;
};

struct HeightBiasRandomWalkFactor {
    HeightBiasRandomWalkFactor(double random_walk_std, double dt)
        : std_(random_walk_std * std::sqrt(std::max(dt, 1.0e-3))) {
    }

    template <typename T>
    bool operator()(const T *const previous, const T *const current, T *residual) const {
        residual[0] = (current[0] - previous[0]) / T(std_);
        return true;
    }

    double std_;
};

struct HeightBiasPriorFactor {
    HeightBiasPriorFactor(double prior, double std)
        : prior_(prior)
        , std_(std) {
    }

    template <typename T>
    bool operator()(const T *const value, T *residual) const {
        residual[0] = (value[0] - T(prior_)) / T(std_);
        return true;
    }

    double prior_;
    double std_;
};

#endif // HEIGHT_BIAS_FACTOR_H
