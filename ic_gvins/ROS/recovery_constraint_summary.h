/*
 * NC-IC extension: small cache helper for async recovery constraint summaries.
 */

#ifndef RECOVERY_CONSTRAINT_SUMMARY_H
#define RECOVERY_CONSTRAINT_SUMMARY_H

#include <array>
#include <cstdint>
#include <vector>

namespace nc_reloc {

enum class RecoveryConstraintSource : int {
    ONLINE_ODOM_RELATIVE = 0,
    IMU_PREINTEGRATION_SUMMARY = 1,
    VISION_RELATIVE_SUMMARY = 2,
    HEADING_RELATIVE = 3,
};

struct RecoveryConstraintSummary {
    int segment_id{-1};
    std::uint64_t node_i{0};
    std::uint64_t node_j{0};
    std::uint32_t revision{0};
    double time_i{0.0};
    double time_j{0.0};
    RecoveryConstraintSource source_type{RecoveryConstraintSource::ONLINE_ODOM_RELATIVE};
    std::array<double, 3> delta_position{{0.0, 0.0, 0.0}};
    double delta_yaw{0.0};
    std::array<double, 4> std{{1.0, 1.0, 1.0, 1.0}};
    double quality_score{1.0};
    int health_state{0};
    bool switchable{false};
};

inline bool sameConstraintKey(const RecoveryConstraintSummary &lhs,
                              const RecoveryConstraintSummary &rhs) {
    return lhs.segment_id == rhs.segment_id &&
           lhs.node_i == rhs.node_i &&
           lhs.node_j == rhs.node_j &&
           lhs.source_type == rhs.source_type;
}

inline bool upsertConstraint(std::vector<RecoveryConstraintSummary> &cache,
                             const RecoveryConstraintSummary &constraint) {
    for (auto &existing : cache) {
        if (!sameConstraintKey(existing, constraint)) {
            continue;
        }
        if (constraint.revision <= existing.revision) {
            return false;
        }
        existing = constraint;
        return true;
    }
    cache.push_back(constraint);
    return true;
}

} // namespace nc_reloc

#endif // RECOVERY_CONSTRAINT_SUMMARY_H
