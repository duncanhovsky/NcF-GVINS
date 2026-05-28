/*
 * NC-IC extension for IC-GVINS.
 *
 * A recovered GNSS observation has two meanings: raw_local is an unbiased map
 * anchor, while the online factor observes the continuous odom trajectory
 * through a fixed segment deviation.  Original IC-GVINS does not require this
 * factor because its online and global frames are identical.
 */

#ifndef RECOVERY_GNSS_FACTOR_H
#define RECOVERY_GNSS_FACTOR_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "common/types.h"
#include "common/rotation.h"

class RecoveryGnssFactor : public ceres::SizedCostFunction<3, 7> {

public:
    explicit RecoveryGnssFactor(GNSS gnss, Vector3d lever)
        : gnss_(std::move(gnss))
        , lever_(std::move(lever)) {
    }

    int activeDof() const {
        return (gnss_.horizontal_valid ? 2 : 0) + (gnss_.vertical_valid ? 1 : 0);
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const Vector3d p{parameters[0][0], parameters[0][1], parameters[0][2]};
        const Quaterniond q{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};
        const Matrix3d yaw_rotation =
            Eigen::AngleAxisd(gnss_.recovery_deviation.yaw, Vector3d::UnitZ()).toRotationMatrix();
        const Vector3d online_measurement =
            yaw_rotation * gnss_.raw_local + gnss_.recovery_deviation.translation;

        Eigen::Map<Eigen::Matrix<double, 3, 1>> error(residuals);
        error = p + q.toRotationMatrix() * lever_ - online_measurement;

        Matrix3d sqrt_info = Matrix3d::Zero();
        if (gnss_.horizontal_valid) {
            sqrt_info(0, 0) = 1.0 / gnss_.std[0];
            sqrt_info(1, 1) = 1.0 / gnss_.std[1];
        }
        if (gnss_.vertical_valid) {
            sqrt_info(2, 2) = 1.0 / gnss_.std[2];
        }
        error = sqrt_info * error;

        if (jacobians && jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
            jacobian_pose.setZero();
            jacobian_pose.block<3, 3>(0, 0) = Matrix3d::Identity();
            jacobian_pose.block<3, 3>(0, 3) =
                -q.toRotationMatrix() * Rotation::skewSymmetric(lever_);
            jacobian_pose = sqrt_info * jacobian_pose;
        }
        return true;
    }

private:
    GNSS gnss_;
    Vector3d lever_;
};

#endif // RECOVERY_GNSS_FACTOR_H
