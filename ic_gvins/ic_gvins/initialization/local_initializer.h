/*
 * NC-IC extension for IC-GVINS.
 *
 * Original IC-GVINS initializes attitude and position from GNSS.  This small
 * initializer supplies a conservative local-odom alternative when GNSS is
 * absent at startup: it requires a verified static IMU interval, fixes local
 * yaw to zero, and leaves later global alignment to the recovery pipeline.
 */

#ifndef LOCAL_INITIALIZER_H
#define LOCAL_INITIALIZER_H

#include "common/types.h"
#include "preintegration/integration_state.h"

#include <deque>

class LocalInitializer {

public:
    struct Options {
        double imu_rate{200.0};
        double static_duration{1.0};
        double gravity{9.80};
    };

    explicit LocalInitializer(const Options &options)
        : options_(options) {
    }

    bool tryInitialize(const std::deque<std::pair<IMU, IntegrationState>> &ins_window,
                       const HeadingObservation *heading, IntegrationState &state) const;

private:
    Options options_;
};

#endif // LOCAL_INITIALIZER_H
