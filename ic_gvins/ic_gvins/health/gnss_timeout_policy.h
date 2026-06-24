/*
 * NC-IC GNSS timeout policy.
 */

#ifndef GNSS_TIMEOUT_POLICY_H
#define GNSS_TIMEOUT_POLICY_H

namespace nc_health {

inline bool shouldTriggerGnssTimeout(bool realtime_mode, double fusion_time,
                                     double last_processed_gnss_time,
                                     double gnss_timeout,
                                     bool has_pending_gnss_before_fusion_time) {
    if (gnss_timeout <= 0.0 || last_processed_gnss_time <= 0.0) {
        return false;
    }
    if (!realtime_mode && has_pending_gnss_before_fusion_time) {
        return false;
    }
    return (fusion_time - last_processed_gnss_time) > gnss_timeout;
}

} // namespace nc_health

#endif // GNSS_TIMEOUT_POLICY_H
