#include "../ic_gvins/ROS/reloc_segment_lifecycle.h"

#include <cassert>
#include <cstdint>
#include <vector>

struct TestNode {
    double time{0.0};
    std::uint64_t node_id{0};
    std::uint32_t revision{0};
    int segment_id{-1};
    bool has_raw_gnss{false};
};

int main() {
    using nc_reloc::RecoveryEventKind;
    using nc_reloc::RelocSegmentLifecycle;
    using nc_reloc::SegmentState;

    RelocSegmentLifecycle<TestNode>::Options options;
    options.maximum_nodes = 100;
    options.min_recovery_anchors = 3;
    options.max_recovery_tail_duration = 100.0;
    RelocSegmentLifecycle<TestNode> lifecycle(options);

    int segment_id = -1;
    std::vector<TestNode> snapshot;

    lifecycle.handleEvent(RecoveryEventKind::DegradedStart, 1, 10.0);
    lifecycle.addNode(TestNode{10.0, 1, 1, 1, true});
    lifecycle.addNode(TestNode{11.0, 2, 1, 1, true});
    assert(!lifecycle.popReadySnapshot(segment_id, snapshot));

    lifecycle.handleEvent(RecoveryEventKind::RecoveryConfirmed, 1, 20.0);
    lifecycle.addNode(TestNode{20.0, 3, 1, 1, true});
    lifecycle.addNode(TestNode{21.0, 4, 1, 1, true});
    assert(!lifecycle.popReadySnapshot(segment_id, snapshot));
    lifecycle.addNode(TestNode{22.0, 5, 1, 1, true});
    assert(lifecycle.popReadySnapshot(segment_id, snapshot));
    assert(segment_id == 1);
    assert(snapshot.size() == 5);
    assert(lifecycle.state(1) == SegmentState::Solving);

    lifecycle.addNode(TestNode{23.0, 6, 1, 1, true});
    assert(!lifecycle.popReadySnapshot(segment_id, snapshot));
    lifecycle.markSolved(1);
    assert(lifecycle.state(1) == SegmentState::Done);
    lifecycle.addNode(TestNode{24.0, 7, 1, 1, true});
    assert(lifecycle.nodeCount(1) == 5);

    lifecycle.handleEvent(RecoveryEventKind::DegradedStart, 2, 30.0);
    lifecycle.addNode(TestNode{30.0, 30, 1, 2, true});
    lifecycle.handleEvent(RecoveryEventKind::RecoveryConfirmed, 2, 40.0);
    lifecycle.addNode(TestNode{40.0, 31, 1, 2, true});
    lifecycle.addNode(TestNode{41.0, 32, 1, 2, true});
    lifecycle.addNode(TestNode{42.0, 33, 1, 2, true});
    assert(lifecycle.popReadySnapshot(segment_id, snapshot));
    assert(segment_id == 2);
    assert(snapshot.size() == 4);

    return 0;
}
