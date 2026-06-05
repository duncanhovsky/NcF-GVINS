/*
 * NC-IC extension: multi-source relative-motion trend arbitration.
 *
 * This header is intentionally ROS/Eigen-free so the health manager can use
 * it as a small consensus primitive and tests can compile without catkin.
 */

#ifndef MOTION_TREND_CONSENSUS_H
#define MOTION_TREND_CONSENSUS_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace nc_health {

enum class MotionTrendSource : int {
    GNSS = 0,
    IMU_PREINTEGRATION = 1,
    VISION_RELATIVE = 2,
    HEADING = 3,
    ONLINE_ODOM = 4,
    WHEEL_ODOM = 5,
};

struct MotionTrendObservation {
    MotionTrendSource source_type{MotionTrendSource::ONLINE_ODOM};
    int independence_group{0};
    double time_i{0.0};
    double time_j{0.0};
    double delta_x{0.0};
    double delta_y{0.0};
    double delta_z{0.0};
    double delta_yaw{0.0};
    double horizontal_std{1.0};
    double vertical_std{1.0};
    double yaw_std{1.0};
    bool horizontal_valid{false};
    bool vertical_valid{false};
    bool yaw_valid{false};
    double quality_score{1.0};

    static MotionTrendObservation horizontal(MotionTrendSource source, int group,
                                             double start_time, double end_time,
                                             double dx, double dy, double std) {
        MotionTrendObservation observation;
        observation.source_type = source;
        observation.independence_group = group;
        observation.time_i = start_time;
        observation.time_j = end_time;
        observation.delta_x = dx;
        observation.delta_y = dy;
        observation.horizontal_std = std;
        observation.horizontal_valid = true;
        return observation;
    }
};

struct TrendConsensusOptions {
    int min_independent_sources{3};
    double horizontal_threshold{3.0};
    double min_horizontal_motion{0.05};
};

struct TrendHealthEvidence {
    bool has_evidence{false};
    bool is_outlier{false};
    MotionTrendSource source_type{MotionTrendSource::ONLINE_ODOM};
    int supporting_sources{0};
    int independent_sources{0};
    double residual{0.0};
    double confidence{0.0};
};

namespace detail {

inline bool finiteHorizontal(const MotionTrendObservation &observation) {
    return observation.horizontal_valid &&
           std::isfinite(observation.delta_x) &&
           std::isfinite(observation.delta_y) &&
           std::isfinite(observation.horizontal_std) &&
           observation.horizontal_std > 0.0 &&
           observation.quality_score > 0.0;
}

inline double horizontalNorm(const MotionTrendObservation &observation) {
    return std::sqrt(observation.delta_x * observation.delta_x +
                     observation.delta_y * observation.delta_y);
}

inline double horizontalResidual(const MotionTrendObservation &lhs,
                                 const MotionTrendObservation &rhs) {
    const double dx = lhs.delta_x - rhs.delta_x;
    const double dy = lhs.delta_y - rhs.delta_y;
    return std::sqrt(dx * dx + dy * dy);
}

inline bool sameIndependentGroup(const MotionTrendObservation &lhs,
                                 const MotionTrendObservation &rhs) {
    return lhs.independence_group == rhs.independence_group;
}

inline int countIndependentGroups(const std::vector<MotionTrendObservation> &observations) {
    std::vector<int> groups;
    for (const auto &observation : observations) {
        if (!finiteHorizontal(observation)) {
            continue;
        }
        if (std::find(groups.begin(), groups.end(), observation.independence_group) ==
            groups.end()) {
            groups.push_back(observation.independence_group);
        }
    }
    return static_cast<int>(groups.size());
}

inline int countIndependentHorizontalSupporters(
    const std::vector<MotionTrendObservation> &observations,
    const MotionTrendObservation &target,
    const MotionTrendObservation &reference,
    double threshold) {
    std::vector<int> supporting_groups;
    for (const auto &candidate : observations) {
        if (sameIndependentGroup(target, candidate)) {
            continue;
        }
        if (horizontalResidual(candidate, reference) > threshold) {
            continue;
        }
        if (std::find(supporting_groups.begin(), supporting_groups.end(),
                      candidate.independence_group) != supporting_groups.end()) {
            continue;
        }
        supporting_groups.push_back(candidate.independence_group);
    }
    return static_cast<int>(supporting_groups.size());
}

} // namespace detail

inline TrendHealthEvidence evaluateSourceHorizontalConsensus(
    const std::vector<MotionTrendObservation> &observations,
    MotionTrendSource target_source,
    const TrendConsensusOptions &options) {
    TrendHealthEvidence evidence;
    evidence.source_type = target_source;

    std::vector<MotionTrendObservation> valid;
    valid.reserve(observations.size());
    for (const auto &observation : observations) {
        if (!detail::finiteHorizontal(observation)) {
            continue;
        }
        if (detail::horizontalNorm(observation) < options.min_horizontal_motion) {
            continue;
        }
        valid.push_back(observation);
    }

    evidence.independent_sources = detail::countIndependentGroups(valid);
    if (evidence.independent_sources < std::max(options.min_independent_sources, 1)) {
        return evidence;
    }

    int best_index = -1;
    int best_supporters = -1;
    for (size_t i = 0; i < valid.size(); i++) {
        const int supporters = detail::countIndependentHorizontalSupporters(
            valid, valid[i], valid[i], options.horizontal_threshold);
        if (supporters > best_supporters) {
            best_supporters = supporters;
            best_index = static_cast<int>(i);
        }
    }

    if (best_index < 0 || best_supporters < 1) {
        return evidence;
    }

    const auto &consensus = valid[static_cast<size_t>(best_index)];
    double target_residual = std::numeric_limits<double>::infinity();
    int target_supporters = 0;
    bool found_target = false;
    for (const auto &observation : valid) {
        if (observation.source_type != target_source) {
            continue;
        }
        found_target = true;
        target_residual = detail::horizontalResidual(observation, consensus);
        target_supporters = detail::countIndependentHorizontalSupporters(
            valid, observation, consensus, options.horizontal_threshold);
        break;
    }
    if (!found_target) {
        return evidence;
    }

    evidence.has_evidence = true;
    evidence.supporting_sources = target_supporters;
    evidence.residual = target_residual;
    evidence.is_outlier =
        target_residual > options.horizontal_threshold &&
        target_supporters >= std::max(options.min_independent_sources - 1, 1);
    evidence.confidence =
        static_cast<double>(target_supporters + 1) /
        static_cast<double>(std::max(evidence.independent_sources, 1));
    return evidence;
}

} // namespace nc_health

#endif // MOTION_TREND_CONSENSUS_H
