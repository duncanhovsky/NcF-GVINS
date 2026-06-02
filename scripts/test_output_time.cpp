#include "../ic_gvins/ic_gvins/common/output_time.h"

#include <cassert>
#include <cmath>

int main() {
    constexpr double gps_week_second = 184126.25;
    constexpr double unix_time = 1717334131.25;
    constexpr double gps_unix_offset = unix_time - gps_week_second;

    const double converted =
        nc_output::unixTimeFromGpsWeekSecond(gps_week_second, gps_unix_offset);

    assert(std::abs(converted - unix_time) < 1.0e-9);
    return 0;
}
