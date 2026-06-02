#ifndef IC_GVINS_ROS_RELOC_SEGMENT_LIFECYCLE_H
#define IC_GVINS_ROS_RELOC_SEGMENT_LIFECYCLE_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>

namespace nc_reloc {

enum class RecoveryEventKind : int {
    DegradedStart = 0,
    RecoveryConfirmed = 1,
    SegmentClosed = 2,
    GlobalAligned = 3,
};

enum class SegmentState {
    Idle,
    Capturing,
    CollectingRecoveryAnchors,
    ReadyToSolve,
    Solving,
    Done,
    Failed,
};

template <typename NodeData>
class RelocSegmentLifecycle {
public:
    struct Options {
        std::size_t maximum_nodes{5000};
        int min_recovery_anchors{10};
        double max_recovery_tail_duration{10.0};
    };

    explicit RelocSegmentLifecycle(const Options &options = Options())
        : options_(options) {
    }

    void handleEvent(RecoveryEventKind event, int segment_id, double time) {
        if (segment_id < 0) {
            return;
        }

        auto &task = tasks_[segment_id];
        task.segment_id = segment_id;
        switch (event) {
        case RecoveryEventKind::DegradedStart:
            if (task.state == SegmentState::Idle) {
                task.state = SegmentState::Capturing;
            }
            break;
        case RecoveryEventKind::RecoveryConfirmed:
        case RecoveryEventKind::GlobalAligned:
            if (task.state == SegmentState::Idle) {
                task.state = SegmentState::Capturing;
            }
            if (canCapture(task.state)) {
                task.state = SegmentState::CollectingRecoveryAnchors;
                task.recovery_confirmed = true;
                task.recovery_time = time;
                queueIfReady(task, false);
            }
            break;
        case RecoveryEventKind::SegmentClosed:
            task.closed = true;
            if (task.state == SegmentState::CollectingRecoveryAnchors) {
                queueIfReady(task, true);
                if (task.state == SegmentState::CollectingRecoveryAnchors) {
                    task.state = SegmentState::Failed;
                }
            }
            break;
        }
    }

    bool addNode(const NodeData &node) {
        if (node.segment_id < 0) {
            return false;
        }

        auto &task = tasks_[node.segment_id];
        task.segment_id = node.segment_id;
        if (task.state == SegmentState::Idle) {
            task.state = SegmentState::Capturing;
        }
        if (!canCapture(task.state)) {
            return false;
        }

        const auto existing = task.nodes.find(node.node_id);
        if (existing != task.nodes.end() && existing->second.revision > node.revision) {
            return false;
        }
        task.nodes[node.node_id] = node;
        task.latest_time = std::max(task.latest_time, node.time);
        while (task.nodes.size() > options_.maximum_nodes) {
            task.nodes.erase(task.nodes.begin());
        }
        queueIfReady(task, false);
        return true;
    }

    bool popReadySnapshot(int &segment_id, std::vector<NodeData> &snapshot) {
        while (!ready_queue_.empty()) {
            const int ready_segment_id = ready_queue_.front();
            ready_queue_.pop_front();
            auto it = tasks_.find(ready_segment_id);
            if (it == tasks_.end() || it->second.state != SegmentState::ReadyToSolve) {
                continue;
            }

            segment_id = ready_segment_id;
            snapshot.clear();
            snapshot.reserve(it->second.nodes.size());
            for (const auto &node : it->second.nodes) {
                snapshot.push_back(node.second);
            }
            it->second.state = SegmentState::Solving;
            return true;
        }
        return false;
    }

    bool hasReady() const {
        for (const int segment_id : ready_queue_) {
            const auto it = tasks_.find(segment_id);
            if (it != tasks_.end() && it->second.state == SegmentState::ReadyToSolve) {
                return true;
            }
        }
        return false;
    }

    void markSolved(int segment_id) {
        auto it = tasks_.find(segment_id);
        if (it != tasks_.end()) {
            it->second.state = SegmentState::Done;
        }
    }

    void markFailed(int segment_id) {
        auto it = tasks_.find(segment_id);
        if (it != tasks_.end()) {
            it->second.state = SegmentState::Failed;
        }
    }

    SegmentState state(int segment_id) const {
        const auto it = tasks_.find(segment_id);
        return it == tasks_.end() ? SegmentState::Idle : it->second.state;
    }

    std::size_t nodeCount(int segment_id) const {
        const auto it = tasks_.find(segment_id);
        return it == tasks_.end() ? 0 : it->second.nodes.size();
    }

    int recoveryAnchorCount(int segment_id) const {
        const auto it = tasks_.find(segment_id);
        return it == tasks_.end() ? 0 : recoveryAnchorCount(it->second);
    }

private:
    struct SegmentTask {
        int segment_id{-1};
        SegmentState state{SegmentState::Idle};
        std::map<std::uint64_t, NodeData> nodes;
        bool recovery_confirmed{false};
        bool closed{false};
        bool queued{false};
        double recovery_time{0.0};
        double latest_time{0.0};
    };

    static bool canCapture(SegmentState state) {
        return state == SegmentState::Idle ||
               state == SegmentState::Capturing ||
               state == SegmentState::CollectingRecoveryAnchors;
    }

    int recoveryAnchorCount(const SegmentTask &task) const {
        int count = 0;
        for (const auto &node : task.nodes) {
            if (node.second.has_raw_gnss && node.second.time >= task.recovery_time) {
                count++;
            }
        }
        return count;
    }

    void queueIfReady(SegmentTask &task, bool force_close) {
        if (!task.recovery_confirmed || task.queued ||
            task.state != SegmentState::CollectingRecoveryAnchors) {
            return;
        }

        const int anchors = recoveryAnchorCount(task);
        const bool enough_anchors =
            anchors >= std::max(options_.min_recovery_anchors, 1);
        const bool timeout =
            options_.max_recovery_tail_duration > 0.0 &&
            task.latest_time >= task.recovery_time &&
            (task.latest_time - task.recovery_time) >= options_.max_recovery_tail_duration;
        if ((enough_anchors || (timeout && anchors > 0) ||
             (force_close && enough_anchors)) &&
            task.nodes.size() >= 2) {
            task.state = SegmentState::ReadyToSolve;
            task.queued = true;
            ready_queue_.push_back(task.segment_id);
        }
    }

    Options options_;
    std::map<int, SegmentTask> tasks_;
    std::deque<int> ready_queue_;
};

} // namespace nc_reloc

#endif // IC_GVINS_ROS_RELOC_SEGMENT_LIFECYCLE_H
