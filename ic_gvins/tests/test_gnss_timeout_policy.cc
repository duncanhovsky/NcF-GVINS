#include "ic_gvins/health/gnss_timeout_policy.h"

#include <cassert>

int main() {
    using nc_health::shouldTriggerGnssTimeout;

    assert(shouldTriggerGnssTimeout(true, 105.0, 100.0, 2.0, true));
    assert(!shouldTriggerGnssTimeout(false, 105.0, 100.0, 2.0, true));
    assert(shouldTriggerGnssTimeout(false, 105.0, 100.0, 2.0, false));
    assert(!shouldTriggerGnssTimeout(false, 101.0, 100.0, 2.0, false));
    assert(!shouldTriggerGnssTimeout(false, 105.0, 0.0, 2.0, false));
    assert(!shouldTriggerGnssTimeout(false, 105.0, 100.0, 0.0, false));

    return 0;
}
