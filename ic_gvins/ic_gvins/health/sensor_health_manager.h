/*
 * NC-IC extension for IC-GVINS.
 *
 * Original IC-GVINS filters each GNSS point before the optimizer and has no
 * degraded/recovery lifecycle.  This manager now supplies modality-neutral
 * integrity arbitration: INS still propagates the real-time state, while
 * GNSS, vision, calibrated heading and IMU intervals can all be diagnosed
 * and selectively admitted or covariance-inflated.
 */

#ifndef SENSOR_HEALTH_MANAGER_H
#define SENSOR_HEALTH_MANAGER_H

#include "common/types.h"

#include <algorithm>

class SensorHealthManager {

public:
    struct Options {
        bool enabled{false};
        int horizontal_recovery_confirm_samples{3};
        int vertical_recovery_confirm_samples{5};
        double horizontal_recovery_confirm_duration{0.0};
        double vertical_recovery_confirm_duration{0.0};
        int vision_degrade_confirm_frames{2};
        int vision_recovery_confirm_frames{3};
        double vision_degraded_covariance_scale{5.0};
        int heading_recovery_confirm_samples{3};
        double heading_degraded_covariance_scale{10.0};
        int imu_recovery_confirm_intervals{5};
        double imu_degraded_covariance_scale{25.0};
    };

    struct Decision {
        AxisHealth horizontal;
        AxisHealth vertical;
        SensorHealthState state{SensorHealthState::UNAVAILABLE};
        bool accept_online{false};
        bool initialize_deviation{false};

        int activeDof() const {
            return (horizontal.accepted ? 2 : 0) + (vertical.accepted ? 1 : 0);
        }
    };

    SensorHealthManager() = default;

    explicit SensorHealthManager(const Options &options)
        : options_(options) {
    }

    Decision updateGnss(double time, bool horizontal_valid, bool vertical_valid,
                        double horizontal_innovation = 0.0, double vertical_innovation = 0.0) {
        const SensorHealthState previous_horizontal = horizontal_.state;

        if (!options_.enabled) {
            horizontal_ = passThrough(time, horizontal_valid, horizontal_innovation);
            vertical_ = passThrough(time, vertical_valid, vertical_innovation);
        } else {
            updateAxis(horizontal_valid, options_.horizontal_recovery_confirm_samples,
                       options_.horizontal_recovery_confirm_duration, time, horizontal_innovation,
                       1.0, false, horizontal_);
            updateAxis(vertical_valid, options_.vertical_recovery_confirm_samples,
                       options_.vertical_recovery_confirm_duration, time, vertical_innovation,
                       1.0, false, vertical_);
        }

        Decision decision;
        decision.horizontal = horizontal_;
        decision.vertical = vertical_;
        decision.state = horizontal_.state;
        decision.accept_online = horizontal_.accepted;
        decision.initialize_deviation =
            horizontal_valid &&
            (previous_horizontal == SensorHealthState::DEGRADED ||
             previous_horizontal == SensorHealthState::UNAVAILABLE) &&
            horizontal_.state == SensorHealthState::RECOVERING;
        return decision;
    }

    ModalityDecision updateVision(double time, bool valid, double outlier_ratio) {
        if (!options_.enabled) {
            vision_ = passThrough(time, valid, outlier_ratio);
        } else {
            if (!valid) {
                vision_bad_frames_++;
            } else {
                vision_bad_frames_ = 0;
            }
            const bool admitted_quality =
                valid || vision_bad_frames_ < std::max(options_.vision_degrade_confirm_frames, 1);
            updateAxis(admitted_quality, options_.vision_recovery_confirm_frames, 0.0, time,
                       outlier_ratio, options_.vision_degraded_covariance_scale, true, vision_);
        }
        return makeWeightedDecision(vision_, true);
    }

    ModalityDecision updateHeading(double time, bool valid, double innovation) {
        if (!options_.enabled) {
            heading_ = passThrough(time, valid, innovation);
        } else {
            updateAxis(valid, options_.heading_recovery_confirm_samples, 0.0, time, innovation,
                       options_.heading_degraded_covariance_scale, false, heading_);
        }
        return makeWeightedDecision(heading_, false);
    }

    ModalityDecision updateImu(double time, bool valid, double diagnostic) {
        if (!options_.enabled) {
            imu_ = passThrough(time, valid, diagnostic);
        } else {
            updateAxis(valid, options_.imu_recovery_confirm_intervals, 0.0, time, diagnostic,
                       options_.imu_degraded_covariance_scale, true, imu_);
        }
        // Propagation time cannot be removed from a single-IMU estimator.
        return makeWeightedDecision(imu_, true);
    }

    // NC-IC extension: local-bootstrap odom has no initial global alignment.
    // The first later GNSS samples must therefore follow the recovery path,
    // even if their sensor covariance is healthy.
    void forceGnssDegraded() {
        horizontal_.state = SensorHealthState::DEGRADED;
        horizontal_.recovery_samples = 0;
        horizontal_.accepted = false;
        vertical_.state = SensorHealthState::DEGRADED;
        vertical_.recovery_samples = 0;
        vertical_.accepted = false;
        horizontal_.covariance_scale = 1.0;
        vertical_.covariance_scale = 1.0;
    }

    SensorHealthState gnssState() const {
        return horizontal_.state;
    }

    SensorHealthState horizontalState() const {
        return horizontal_.state;
    }

    SensorHealthState verticalState() const {
        return vertical_.state;
    }

    SensorHealthState visionState() const {
        return vision_.state;
    }

    SensorHealthState headingState() const {
        return heading_.state;
    }

    SensorHealthState imuState() const {
        return imu_.state;
    }

    bool enabled() const {
        return options_.enabled;
    }

private:
    static AxisHealth passThrough(double time, bool valid, double innovation) {
        AxisHealth axis;
        axis.state = valid ? SensorHealthState::ACTIVE : SensorHealthState::DEGRADED;
        axis.innovation = innovation;
        axis.accepted = valid;
        axis.last_time = time;
        axis.covariance_scale = valid ? 1.0 : 25.0;
        return axis;
    }

    static ModalityDecision makeWeightedDecision(const AxisHealth &health, bool retain_when_degraded) {
        ModalityDecision decision;
        decision.health = health;
        decision.admit = health.accepted || retain_when_degraded;
        decision.covariance_scale = health.covariance_scale;
        return decision;
    }

    static void updateAxis(bool valid, int confirm_samples, double confirm_duration, double time,
                           double innovation, double degraded_scale, bool retain_when_degraded,
                           AxisHealth &axis) {
        axis.innovation = innovation;
        axis.last_time = time;
        if (!valid) {
            axis.state = SensorHealthState::DEGRADED;
            axis.recovery_samples = 0;
            axis.recovery_start_time = 0;
            axis.accepted = retain_when_degraded;
            axis.covariance_scale = std::max(degraded_scale, 1.0);
            return;
        }

        if (axis.state == SensorHealthState::DEGRADED ||
            axis.state == SensorHealthState::RECOVERING) {
            if (axis.state == SensorHealthState::DEGRADED) {
                axis.recovery_start_time = time;
            }
            axis.recovery_samples++;
            axis.state = SensorHealthState::RECOVERING;
            axis.accepted = retain_when_degraded;
            axis.covariance_scale = std::max(degraded_scale, 1.0);
            const bool enough_samples = axis.recovery_samples >= std::max(confirm_samples, 1);
            const bool enough_time =
                confirm_duration <= 0.0 || (time - axis.recovery_start_time) >= confirm_duration;
            if (enough_samples && enough_time) {
                axis.state = SensorHealthState::ACTIVE;
                axis.accepted = true;
                axis.covariance_scale = 1.0;
            }
            return;
        }

        axis.state = SensorHealthState::ACTIVE;
        axis.accepted = true;
        axis.covariance_scale = 1.0;
    }

private:
    Options options_;
    AxisHealth horizontal_;
    AxisHealth vertical_;
    AxisHealth vision_;
    AxisHealth heading_;
    AxisHealth imu_;
    int vision_bad_frames_{0};
};

#endif // SENSOR_HEALTH_MANAGER_H
