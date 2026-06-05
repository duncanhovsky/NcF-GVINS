#include "ic_gvins/visualization/gnss_status_color.h"

namespace {

constexpr bool sameColor(const nc_visualization::StatusColor &color,
                         double r, double g, double b, double a) {
    return color.r == r && color.g == g && color.b == b && color.a == a;
}

} // namespace

static_assert(sameColor(nc_visualization::gnssStatusColor(1, true, true, false, false),
                        0.05, 0.85, 0.20, 1.0),
              "healthy GNSS should be green");
static_assert(sameColor(nc_visualization::gnssStatusColor(2, true, true, false, false),
                        0.95, 0.05, 0.05, 1.0),
              "degraded GNSS should be red");
static_assert(sameColor(nc_visualization::gnssStatusColor(1, true, true, true, false),
                        0.95, 0.05, 0.05, 1.0),
              "forced degraded GNSS should be red");
static_assert(sameColor(nc_visualization::gnssStatusColor(3, true, true, false, true),
                        0.02, 0.12, 0.55, 1.0),
              "recovery anchors should be deep blue");
static_assert(sameColor(nc_visualization::gnssStatusColor(3, true, true, true, false),
                        0.95, 0.05, 0.05, 1.0),
              "forced degraded recovering GNSS should stay red unless it is an anchor");

int main() {
    return 0;
}
