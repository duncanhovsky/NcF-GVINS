#include "ROS/recovery_constraint_summary.h"

#include <cassert>
#include <vector>

int main() {
    nc_reloc::RecoveryConstraintSummary first;
    first.segment_id = 4;
    first.node_i = 100;
    first.node_j = 200;
    first.source_type = nc_reloc::RecoveryConstraintSource::ONLINE_ODOM_RELATIVE;
    first.revision = 1;
    first.delta_position = {1.0, 0.0, 0.0};

    nc_reloc::RecoveryConstraintSummary updated = first;
    updated.revision = 2;
    updated.delta_position = {2.0, 0.0, 0.0};

    std::vector<nc_reloc::RecoveryConstraintSummary> cache;
    assert(nc_reloc::upsertConstraint(cache, first));
    assert(!nc_reloc::upsertConstraint(cache, first));
    assert(nc_reloc::upsertConstraint(cache, updated));
    assert(cache.size() == 1);
    assert(cache.front().revision == 2);
    assert(cache.front().delta_position[0] == 2.0);

    return 0;
}
