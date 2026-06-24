#include "ic_gvins/diagnostics/console_diagnostics.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool contains(const std::string &text, const std::string &needle) {
    return text.find(needle) != std::string::npos;
}

void requireContains(const std::string &text, const std::string &needle) {
    if (!contains(text, needle)) {
        std::cerr << "missing substring: " << needle << "\ntext was:\n" << text << std::endl;
        std::exit(1);
    }
}

void testHealthLineShowsDecisionValues() {
    nc_diag::HealthDecisionLine line;
    line.time = 123.456;
    line.sensor = "GNSS-H";
    line.previous_state = "active";
    line.current_state = "degraded";
    line.accepted = false;
    line.innovation = 24.5;
    line.threshold = 20.0;
    line.reason = "horizontal innovation";

    const std::string text = nc_diag::formatHealthDecision(line);

    requireContains(text, "[HEALTH]");
    requireContains(text, "GNSS-H");
    requireContains(text, "active->degraded");
    requireContains(text, "accept=N");
    requireContains(text, "innov=24.50/20.00");
    requireContains(text, "horizontal innovation");
}

void testOptimizationBlockIsCompactAndShowsSlots() {
    nc_diag::OptimizationWindowLog log;
    log.sequence = 7;
    log.time = 321.0;
    log.state_count = 3;
    log.keyframe_count = 2;
    log.preintegration_count = 2;
    log.gnss_factor_count = 1;
    log.heading_factor_count = 1;
    log.visual_factor_count = 42;
    log.visual_outlier_count = 4;
    log.gnss_reweighted_count = 1;
    log.first_iterations = 2;
    log.second_iterations = 3;
    log.first_cost_ms = 1.5;
    log.second_cost_ms = 2.5;

    nc_diag::PoseBrief before;
    before.time = 320.0;
    before.p[0] = 1.0;
    before.p[1] = 2.0;
    before.p[2] = -0.5;
    before.yaw_deg = 10.0;
    log.before.push_back(before);
    nc_diag::PoseBrief after = before;
    after.p[0] = 1.2;
    after.yaw_deg = 11.0;
    log.after.push_back(after);

    nc_diag::SlotFactorLog slot;
    slot.index = 0;
    slot.time = 320.0;
    slot.imu_to_next = true;
    slot.gnss_count = 1;
    slot.gnss_chi2 = 9.0;
    slot.gnss_threshold = 7.815;
    slot.gnss_reweighted = true;
    slot.gnss_scale = 1.07;
    slot.visual_count = 12;
    slot.visual_outliers = 2;
    slot.heading_count = 1;
    log.slots.push_back(slot);

    const std::string text = nc_diag::formatOptimizationWindow(log);

    requireContains(text, "[OPT #7]");
    requireContains(text, "win=3");
    requireContains(text, "gnss=1 rw=1");
    requireContains(text, "visual=42 out=4");
    requireContains(text, "before");
    requireContains(text, "after");
    requireContains(text, "slot 00");
    requireContains(text, "G:1 chi2=9.00/7.82 RWx1.07");
    requireContains(text, "pose (1.00,2.00,-0.50)->(1.20,2.00,-0.50)");
    requireContains(text, "yaw=10.0->11.0");
}

} // namespace

int main() {
    testHealthLineShowsDecisionValues();
    testOptimizationBlockIsCompactAndShowsSlots();
    std::cout << "console_diagnostics_test passed" << std::endl;
    return 0;
}
