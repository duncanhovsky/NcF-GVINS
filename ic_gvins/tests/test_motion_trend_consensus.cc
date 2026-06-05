#include "ic_gvins/health/motion_trend_consensus.h"

#include <cassert>
#include <vector>

int main() {
    using nc_health::MotionTrendObservation;
    using nc_health::MotionTrendSource;
    using nc_health::TrendConsensusOptions;

    TrendConsensusOptions options;
    options.min_independent_sources = 3;
    options.horizontal_threshold = 2.0;
    options.min_horizontal_motion = 0.1;

    std::vector<MotionTrendObservation> observations;
    observations.push_back(MotionTrendObservation::horizontal(
        MotionTrendSource::GNSS, 1, 10.0, 11.0, 11.0, 0.0, 1.0));
    observations.push_back(MotionTrendObservation::horizontal(
        MotionTrendSource::IMU_PREINTEGRATION, 2, 10.0, 11.0, 1.0, 0.0, 1.0));
    observations.push_back(MotionTrendObservation::horizontal(
        MotionTrendSource::VISION_RELATIVE, 3, 10.0, 11.0, 1.1, 0.1, 1.0));

    const auto evidence = nc_health::evaluateSourceHorizontalConsensus(
        observations, MotionTrendSource::GNSS, options);
    assert(evidence.has_evidence);
    assert(evidence.is_outlier);
    assert(evidence.supporting_sources == 2);
    assert(evidence.independent_sources == 3);

    observations.pop_back();
    const auto insufficient = nc_health::evaluateSourceHorizontalConsensus(
        observations, MotionTrendSource::GNSS, options);
    assert(!insufficient.has_evidence);
    assert(!insufficient.is_outlier);

    return 0;
}
