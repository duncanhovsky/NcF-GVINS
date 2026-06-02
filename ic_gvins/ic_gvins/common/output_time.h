#ifndef GVINS_OUTPUT_TIME_H
#define GVINS_OUTPUT_TIME_H

namespace nc_output {

inline double unixTimeFromGpsWeekSecond(double gps_week_second, double gps_unix_offset) {
    return gps_week_second + gps_unix_offset;
}

} // namespace nc_output

#endif // GVINS_OUTPUT_TIME_H
