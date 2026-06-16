#include "ic_gvins/misc.h"

#include <cassert>
#include <deque>
#include <utility>
#include <vector>

namespace {

std::pair<IMU, IntegrationState> makeInsSample(double time) {
    IMU imu;
    imu.time = time;
    imu.dt = 0.1;

    IntegrationState state;
    state.time = time;

    return {imu, state};
}

} // namespace

int main() {
    std::deque<std::pair<IMU, IntegrationState>> window;
    window.push_back(makeInsSample(10.0));
    window.push_back(makeInsSample(10.1));
    window.push_back(makeInsSample(10.2));

    std::deque<IMU> history;
    for (const auto &sample : window) {
        history.push_back(sample.first);
    }

    std::vector<IMU> series;

    assert(!MISC::getImuSeriesFromTo(window, 9.9, 10.05, series));
    assert(series.empty());

    assert(!MISC::getImuSeriesFromTo(window, 10.05, 10.25, series));
    assert(series.empty());

    assert(MISC::getImuSeriesFromTo(window, 10.05, 10.15, series));
    assert(!series.empty());

    std::deque<std::pair<IMU, IntegrationState>> trimmed_window;
    trimmed_window.push_back(makeInsSample(10.1));
    trimmed_window.push_back(makeInsSample(10.2));

    assert(!MISC::getImuSeriesFromTo(trimmed_window, 10.05, 10.15, series));
    assert(series.empty());

    assert(MISC::getImuSeriesFromTo(history, 10.05, 10.15, series));
    assert(!series.empty());

    return 0;
}
