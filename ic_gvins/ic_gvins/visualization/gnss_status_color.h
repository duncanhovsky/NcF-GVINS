/*
 * NC-IC extension: shared GNSS status colors for ROS/RViz diagnostics.
 */

#ifndef GNSS_STATUS_COLOR_H
#define GNSS_STATUS_COLOR_H

namespace nc_visualization {

struct StatusColor {
    double r{1.0};
    double g{1.0};
    double b{1.0};
    double a{1.0};
};

constexpr StatusColor gnssStatusColor(int health_state, bool horizontal_valid,
                                      bool vertical_valid, bool forced_degraded,
                                      bool recovery_anchor) {
    if (recovery_anchor || health_state == 3) {
        if (!forced_degraded && horizontal_valid && vertical_valid) {
            return {0.02, 0.12, 0.55, 1.0};
        }
    }
    if (forced_degraded || !horizontal_valid || !vertical_valid || health_state == 2) {
        return {0.95, 0.05, 0.05, 1.0};
    }
    return {0.05, 0.85, 0.20, 1.0};
}

} // namespace nc_visualization

#endif // GNSS_STATUS_COLOR_H
