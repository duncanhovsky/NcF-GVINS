#ifndef CONSOLE_DIAGNOSTICS_H
#define CONSOLE_DIAGNOSTICS_H

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace nc_diag {

struct ConsoleDiagnosticsOptions {
    bool enabled{true};
    int window_detail_stride{1};
    int max_window_slots{0};
};

struct HealthDecisionLine {
    double time{0.0};
    std::string sensor;
    std::string previous_state;
    std::string current_state;
    bool accepted{false};
    double innovation{0.0};
    double threshold{0.0};
    int recovery_samples{0};
    std::string reason;
};

struct MotionTrendLine {
    double time{0.0};
    std::string source;
    bool has_evidence{false};
    bool is_outlier{false};
    int supporting_sources{0};
    int independent_sources{0};
    double residual{0.0};
    double threshold{0.0};
};

struct PoseBrief {
    double time{0.0};
    double p[3]{0.0, 0.0, 0.0};
    double yaw_deg{0.0};
};

struct SlotFactorLog {
    int index{0};
    double time{0.0};
    bool keyframe{false};
    bool imu_to_next{false};
    bool imu_chi2_valid{false};
    double imu_chi2{0.0};
    int gnss_count{0};
    double gnss_chi2{0.0};
    double gnss_threshold{0.0};
    bool gnss_reweighted{false};
    double gnss_scale{1.0};
    int visual_count{0};
    int visual_outliers{0};
    int heading_count{0};
};

struct GnssFactorDecision {
    double time{0.0};
    int state_index{-1};
    int dof{0};
    double chi2{0.0};
    double threshold{0.0};
    bool reweighted{false};
    double scale{1.0};
};

struct VisualResidualDecision {
    int ref_index{-1};
    int obs_index{-1};
    bool removed{false};
};

struct OptimizationWindowLog {
    int sequence{0};
    double time{0.0};
    int state_count{0};
    int keyframe_count{0};
    int preintegration_count{0};
    int gnss_factor_count{0};
    int heading_factor_count{0};
    int visual_factor_count{0};
    int visual_outlier_count{0};
    int gnss_reweighted_count{0};
    int first_iterations{0};
    int second_iterations{0};
    double first_cost_ms{0.0};
    double second_cost_ms{0.0};
    double marginalization_cost_ms{0.0};
    std::vector<PoseBrief> before;
    std::vector<PoseBrief> after;
    std::vector<SlotFactorLog> slots;
};

inline std::string yesNo(bool value) {
    return value ? "Y" : "N";
}

inline std::string fixed(double value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

inline std::string formatVec3(const double p[3]) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "(" << p[0] << "," << p[1] << "," << p[2] << ")";
    return stream.str();
}

inline std::string formatPoseBrief(const PoseBrief &pose) {
    std::ostringstream stream;
    stream << "t=" << fixed(pose.time, 3)
           << " p=" << formatVec3(pose.p)
           << " yaw=" << fixed(pose.yaw_deg, 1);
    return stream.str();
}

inline std::string formatHealthDecision(const HealthDecisionLine &line) {
    std::ostringstream stream;
    stream << "[HEALTH] t=" << fixed(line.time, 3)
           << " " << line.sensor << " "
           << line.previous_state << "->" << line.current_state
           << " accept=" << yesNo(line.accepted)
           << " innov=" << fixed(line.innovation, 2) << "/" << fixed(line.threshold, 2);
    if (line.recovery_samples > 0) {
        stream << " samples=" << line.recovery_samples;
    }
    if (!line.reason.empty()) {
        stream << " reason=" << line.reason;
    }
    return stream.str();
}

inline std::string formatMotionTrend(const MotionTrendLine &line) {
    std::ostringstream stream;
    stream << "[TREND] t=" << fixed(line.time, 3)
           << " source=" << line.source
           << " evidence=" << yesNo(line.has_evidence)
           << " outlier=" << yesNo(line.is_outlier)
           << " residual=" << fixed(line.residual, 2) << "/" << fixed(line.threshold, 2)
           << " support=" << line.supporting_sources << "/" << line.independent_sources;
    return stream.str();
}

inline std::string formatSlot(const SlotFactorLog &slot) {
    std::ostringstream stream;
    stream << "  slot " << std::setw(2) << std::setfill('0') << slot.index
           << std::setfill(' ') << " t=" << fixed(slot.time, 3)
           << " KF=" << yesNo(slot.keyframe)
           << " IMU=" << yesNo(slot.imu_to_next);
    if (slot.imu_chi2_valid) {
        stream << "(" << fixed(slot.imu_chi2, 1) << ")";
    }
    stream
           << " G:" << slot.gnss_count;
    if (slot.gnss_count > 0) {
        stream << " chi2=" << fixed(slot.gnss_chi2, 2)
               << "/" << fixed(slot.gnss_threshold, 2);
        if (slot.gnss_reweighted) {
            stream << " RWx" << fixed(slot.gnss_scale, 2);
        }
    }
    stream << " V:" << slot.visual_count;
    if (slot.visual_outliers > 0) {
        stream << " out=" << slot.visual_outliers;
    }
    stream << " H:" << slot.heading_count;
    return stream.str();
}

inline std::string formatOptimizationWindow(const OptimizationWindowLog &log) {
    std::ostringstream stream;
    stream << "[OPT #" << log.sequence << "] t=" << fixed(log.time, 3)
           << " win=" << log.state_count
           << " kf=" << log.keyframe_count
           << " imu=" << log.preintegration_count
           << " gnss=" << log.gnss_factor_count << " rw=" << log.gnss_reweighted_count
           << " visual=" << log.visual_factor_count << " out=" << log.visual_outlier_count
           << " heading=" << log.heading_factor_count
           << " iter=" << log.first_iterations << "+" << log.second_iterations
           << " ms=" << fixed(log.first_cost_ms, 1) << "+" << fixed(log.second_cost_ms, 1);
    if (log.marginalization_cost_ms > 0.0) {
        stream << " marg=" << fixed(log.marginalization_cost_ms, 1);
    }

    const size_t pose_count = std::min(log.before.size(), log.after.size());
    if (pose_count > 0) {
        const auto &first_before = log.before.front();
        const auto &first_after = log.after.front();
        const auto &last_before = log.before.back();
        const auto &last_after = log.after.back();
        stream << "\n  before first " << formatPoseBrief(first_before)
               << " | last " << formatPoseBrief(last_before)
               << "\n  after  first " << formatPoseBrief(first_after)
               << " | last " << formatPoseBrief(last_after);
    }

    for (const auto &slot : log.slots) {
        stream << "\n" << formatSlot(slot);
        if (slot.index >= 0 &&
            static_cast<size_t>(slot.index) < log.before.size() &&
            static_cast<size_t>(slot.index) < log.after.size()) {
            const auto &before = log.before[static_cast<size_t>(slot.index)];
            const auto &after = log.after[static_cast<size_t>(slot.index)];
            stream << " pose " << formatVec3(before.p) << "->" << formatVec3(after.p)
                   << " yaw=" << fixed(before.yaw_deg, 1) << "->" << fixed(after.yaw_deg, 1);
        }
    }
    return stream.str();
}

} // namespace nc_diag

#endif // CONSOLE_DIAGNOSTICS_H
