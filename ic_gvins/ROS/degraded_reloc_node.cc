/*
 * NC-IC extension for IC-GVINS.
 *
 * This asynchronous node follows the role of a VINS-Fusion loop node: the
 * real-time estimator continues to publish a smooth odom trajectory while
 * this process corrects a globally anchored map trajectory.  In contrast to
 * online recovery factors, GNSS anchors here always use unbiased raw_gnss.
 */

#include <ic_gvins/RecoveryEvent.h>
#include <ic_gvins/RecoveryFrame.h>

#include "ic_gvins/common/output_time.h"
#include "reloc_segment_lifecycle.h"

#include <ceres/ceres.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double yawFromQuaternion(const Eigen::Quaterniond &q) {
    const Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
    return std::atan2(rotation(1, 0), rotation(0, 0));
}

Eigen::Quaterniond quaternionFromMessage(const geometry_msgs::Quaternion &q) {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

struct RelocNodeData {
    double time{0};
    std::uint64_t node_id{0};
    std::uint32_t revision{0};
    int segment_id{-1};
    Eigen::Vector3d odom_position{0, 0, 0};
    Eigen::Quaterniond odom_orientation{1, 0, 0, 0};
    double odom_yaw{0};
    Eigen::Vector3d antenna_lever{0, 0, 0};
    bool has_raw_gnss{false};
    bool map_only_anchor{false};
    bool horizontal_valid{true};
    bool vertical_valid{true};
    bool height_bias_valid{false};
    double estimated_height_bias{0};
    int health_state{0};
    Eigen::Vector3d raw_gnss{0, 0, 0};
    Eigen::Vector3d gnss_std{1, 1, 1};
};

struct MapCorrection {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

struct SegmentResult {
    int segment_id{-1};
    double start_time{0};
    double end_time{0};
    MapCorrection correction;
    std::map<std::uint64_t, geometry_msgs::PoseStamped> optimized_poses;
};

struct TrajectoryRow {
    double gps_time{0};
    Eigen::Vector3d position{0, 0, 0};
    Eigen::Quaterniond orientation{1, 0, 0, 0};
};

std::string joinPath(const std::string &directory, const std::string &filename) {
    if (directory.empty()) {
        return filename;
    }
    const char last = directory.back();
    if (last == '/' || last == '\\') {
        return directory + filename;
    }
    return directory + "/" + filename;
}

struct RelativeFourDofFactor {
    RelativeFourDofFactor(const Eigen::Vector3d &delta_position, double delta_yaw,
                          double position_std, double yaw_std)
        : delta_position_(delta_position)
        , delta_yaw_(delta_yaw)
        , position_std_(position_std)
        , yaw_std_(yaw_std) {
    }

    template <typename T>
    bool operator()(const T *const pi, const T *const yi, const T *const pj, const T *const yj,
                    T *residuals) const {
        const T dx = pj[0] - pi[0];
        const T dy = pj[1] - pi[1];
        const T dz = pj[2] - pi[2];
        const T cosine = ceres::cos(yi[0]);
        const T sine = ceres::sin(yi[0]);

        // NC-IC extension: retain IC online relative motion in the corrected
        // map graph instead of forcing historical states back into its window.
        const T local_x = cosine * dx + sine * dy;
        const T local_y = -sine * dx + cosine * dy;
        residuals[0] = (local_x - T(delta_position_.x())) / T(position_std_);
        residuals[1] = (local_y - T(delta_position_.y())) / T(position_std_);
        residuals[2] = (dz - T(delta_position_.z())) / T(position_std_);
        residuals[3] = (yj[0] - yi[0] - T(delta_yaw_)) / T(yaw_std_);
        return true;
    }

    Eigen::Vector3d delta_position_;
    double delta_yaw_;
    double position_std_;
    double yaw_std_;
};

struct RawGnssAnchorFactor {
    RawGnssAnchorFactor(const Eigen::Vector3d &raw_gnss, const Eigen::Vector3d &std,
                        const Eigen::Vector3d &lever_in_odom, double odom_yaw,
                        bool horizontal_valid, bool vertical_valid)
        : raw_gnss_(raw_gnss)
        , std_(std)
        , lever_in_odom_(lever_in_odom)
        , odom_yaw_(odom_yaw)
        , horizontal_valid_(horizontal_valid)
        , vertical_valid_(vertical_valid) {
    }

    template <typename T>
    bool operator()(const T *const position, const T *const yaw, T *residuals) const {
        const T correction_yaw = yaw[0] - T(odom_yaw_);
        const T cosine = ceres::cos(correction_yaw);
        const T sine = ceres::sin(correction_yaw);
        const T lever_x = cosine * T(lever_in_odom_.x()) - sine * T(lever_in_odom_.y());
        const T lever_y = sine * T(lever_in_odom_.x()) + cosine * T(lever_in_odom_.y());

        // NC-IC extension: this factor deliberately ignores online_offset.
        // The asynchronous map must be tied to unbiased raw GNSS rather than
        // to the drift-compatible observation used by online odom.
        residuals[0] = horizontal_valid_
                           ? (position[0] + lever_x - T(raw_gnss_.x())) / T(std_.x())
                           : T(0);
        residuals[1] = horizontal_valid_
                           ? (position[1] + lever_y - T(raw_gnss_.y())) / T(std_.y())
                           : T(0);
        residuals[2] = vertical_valid_
                           ? (position[2] + T(lever_in_odom_.z()) - T(raw_gnss_.z())) / T(std_.z())
                           : T(0);
        return true;
    }

    Eigen::Vector3d raw_gnss_;
    Eigen::Vector3d std_;
    Eigen::Vector3d lever_in_odom_;
    double odom_yaw_;
    bool horizontal_valid_;
    bool vertical_valid_;
};

} // namespace

class DegradedRelocNode {

public:
    DegradedRelocNode()
        : private_node_("~") {
        private_node_.param("relative_position_std", relative_position_std_, 0.15);
        private_node_.param("relative_yaw_std_deg", relative_yaw_std_deg_, 1.0);
        private_node_.param("relative_reference_interval", relative_reference_interval_, 0.5);
        private_node_.param("maximum_nodes", maximum_nodes_, 5000);
        private_node_.param("maximum_visualization_nodes", maximum_visualization_nodes_, 20000);
        private_node_.param("min_recovery_anchors", min_recovery_anchors_, 10);
        private_node_.param("max_recovery_tail_duration", max_recovery_tail_duration_, 10.0);
        private_node_.param("optimize_num_iterations", optimize_num_iterations_, 20);
        private_node_.param("tf_publish_rate", tf_publish_rate_, 20.0);

        nc_reloc::RelocSegmentLifecycle<RelocNodeData>::Options lifecycle_options;
        lifecycle_options.maximum_nodes = static_cast<std::size_t>(maximum_nodes_);
        lifecycle_options.min_recovery_anchors = min_recovery_anchors_;
        lifecycle_options.max_recovery_tail_duration = max_recovery_tail_duration_;
        lifecycle_ = nc_reloc::RelocSegmentLifecycle<RelocNodeData>(lifecycle_options);

        frame_subscriber_ =
            node_.subscribe("recovery_frame", 200, &DegradedRelocNode::frameCallback, this);
        event_subscriber_ =
            node_.subscribe("recovery_event", 20, &DegradedRelocNode::eventCallback, this);
        global_path_publisher_ = node_.advertise<nav_msgs::Path>("global_path", 2);
        path_map_publisher_ = node_.advertise<nav_msgs::Path>("path_map", 2);
        map_to_odom_publisher_ =
            node_.advertise<geometry_msgs::TransformStamped>("map_to_odom", 2);
        reloc_segments_publisher_ =
            node_.advertise<visualization_msgs::MarkerArray>("reloc_segments", 2);

        if (tf_publish_rate_ > 0.0) {
            tf_timer_ = node_.createTimer(ros::Duration(1.0 / tf_publish_rate_),
                                          &DegradedRelocNode::tfTimerCallback, this);
        }

        worker_ = std::thread(&DegradedRelocNode::process, this);
    }

    ~DegradedRelocNode() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            finished_ = true;
            pending_ = true;
        }
        condition_.notify_one();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

private:
    void frameCallback(const ic_gvins::RecoveryFrameConstPtr &message) {
        RelocNodeData data;
        data.time = message->gps_time;
        data.node_id = message->node_id;
        data.revision = message->revision;
        data.segment_id = message->segment_id;
        data.odom_position =
            Eigen::Vector3d(message->odom_pose.position.x, message->odom_pose.position.y,
                            message->odom_pose.position.z);
        data.odom_orientation = quaternionFromMessage(message->odom_pose.orientation);
        data.odom_yaw = yawFromQuaternion(data.odom_orientation);
        data.antenna_lever =
            Eigen::Vector3d(message->antenna_lever.x, message->antenna_lever.y,
                            message->antenna_lever.z);
        data.has_raw_gnss = message->has_raw_gnss;
        data.map_only_anchor = message->map_only_anchor;
        data.horizontal_valid = message->horizontal_valid;
        data.vertical_valid = message->vertical_valid;
        data.height_bias_valid = message->height_bias_valid;
        data.estimated_height_bias = message->estimated_height_bias;
        data.health_state = message->health_state;
        data.raw_gnss =
            Eigen::Vector3d(message->raw_gnss.x, message->raw_gnss.y, message->raw_gnss.z);
        if (data.vertical_valid && data.height_bias_valid) {
            // NC-IC extension: horizontal map anchors remain untouched raw
            // GNSS.  When the explicitly enabled Down-bias model is valid,
            // apply that separately estimated correction only to map height.
            data.raw_gnss.z() += data.estimated_height_bias;
        }
        data.gnss_std = Eigen::Vector3d(std::max(message->gnss_std[0], 1.0e-3),
                                        std::max(message->gnss_std[1], 1.0e-3),
                                        std::max(message->gnss_std[2], 1.0e-3));

        bool should_notify = false;
        bool should_publish_path = false;
        nav_msgs::Path combined_path;
        std::vector<TrajectoryRow> trajectory_rows;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto existing = all_nodes_.find(data.node_id);
            if (existing != all_nodes_.end() && existing->second.revision > data.revision) {
                return;
            }
            all_nodes_[data.node_id] = data;
            while (all_nodes_.size() > static_cast<size_t>(maximum_visualization_nodes_)) {
                all_nodes_.erase(all_nodes_.begin());
            }

            lifecycle_.addNode(data);
            pending_ = lifecycle_.hasReady();
            should_notify = pending_;
            const ros::Time stamp = ros::Time::now();
            combined_path = buildCombinedPathLocked(stamp);
            trajectory_rows = buildTrajectoryRowsLocked(stamp);
            should_publish_path = !combined_path.poses.empty();
        }
        if (should_publish_path) {
            global_path_publisher_.publish(combined_path);
            path_map_publisher_.publish(combined_path);
            writeGlobalPathFiles(trajectory_rows);
        }
        if (should_notify) {
            condition_.notify_one();
        }
    }

    void eventCallback(const ic_gvins::RecoveryEventConstPtr &message) {
        bool should_notify = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            switch (message->event_type) {
            case 0:
                lifecycle_.handleEvent(nc_reloc::RecoveryEventKind::DegradedStart,
                                       message->segment_id, message->gps_time);
                break;
            case 1:
                lifecycle_.handleEvent(nc_reloc::RecoveryEventKind::RecoveryConfirmed,
                                       message->segment_id, message->gps_time);
                break;
            case 2:
                lifecycle_.handleEvent(nc_reloc::RecoveryEventKind::SegmentClosed,
                                       message->segment_id, message->gps_time);
                break;
            case 3:
                lifecycle_.handleEvent(nc_reloc::RecoveryEventKind::GlobalAligned,
                                       message->segment_id, message->gps_time);
                break;
            default:
                break;
            }
            pending_ = lifecycle_.hasReady();
            should_notify = pending_;
        }
        if (should_notify) {
            condition_.notify_one();
        }
    }

    void process() {
        while (ros::ok()) {
            std::vector<RelocNodeData> snapshot;
            int segment_id = -1;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this]() { return pending_ || finished_; });
                if (finished_) {
                    return;
                }
                pending_ = false;
                if (!lifecycle_.popReadySnapshot(segment_id, snapshot)) {
                    continue;
                }
            }
            const bool solved = optimizeAndPublish(segment_id, snapshot);

            bool should_notify = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (solved) {
                    lifecycle_.markSolved(segment_id);
                } else {
                    lifecycle_.markFailed(segment_id);
                }
                pending_ = lifecycle_.hasReady();
                should_notify = pending_;
            }
            if (should_notify) {
                condition_.notify_one();
            }
        }
    }

    bool optimizeAndPublish(int segment_id, const std::vector<RelocNodeData> &nodes) {
        if (nodes.size() < 2) {
            return false;
        }

        size_t gnss_anchors = 0;
        for (const auto &node : nodes) {
            if (node.has_raw_gnss) {
                gnss_anchors++;
            }
        }
        if (gnss_anchors < static_cast<size_t>(std::max(min_recovery_anchors_, 1))) {
            return false;
        }

        std::vector<std::array<double, 3>> positions(nodes.size());
        std::vector<double> yaws(nodes.size());
        ceres::Problem problem;
        for (size_t i = 0; i < nodes.size(); i++) {
            positions[i] = {nodes[i].odom_position.x(), nodes[i].odom_position.y(),
                            nodes[i].odom_position.z()};
            yaws[i] = nodes[i].odom_yaw;
            problem.AddParameterBlock(positions[i].data(), 3);
            problem.AddParameterBlock(&yaws[i], 1);
        }

        const double relative_yaw_std = relative_yaw_std_deg_ * M_PI / 180.0;
        for (size_t i = 1; i < nodes.size(); i++) {
            const Eigen::Matrix3d rotation =
                Eigen::AngleAxisd(-nodes[i - 1].odom_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            const Eigen::Vector3d relative_position =
                rotation * (nodes[i].odom_position - nodes[i - 1].odom_position);
            const double relative_yaw = normalizeAngle(nodes[i].odom_yaw - nodes[i - 1].odom_yaw);
            const double interval_scale =
                std::sqrt(std::max((nodes[i].time - nodes[i - 1].time) /
                                       std::max(relative_reference_interval_, 1.0e-3),
                                   1.0));
            auto *factor = new ceres::AutoDiffCostFunction<RelativeFourDofFactor, 4, 3, 1, 3, 1>(
                new RelativeFourDofFactor(relative_position, relative_yaw,
                                          relative_position_std_ * interval_scale,
                                          relative_yaw_std * interval_scale));
            problem.AddResidualBlock(factor, nullptr, positions[i - 1].data(), &yaws[i - 1],
                                     positions[i].data(), &yaws[i]);
        }

        auto *gnss_loss = new ceres::HuberLoss(1.0);
        for (size_t i = 0; i < nodes.size(); i++) {
            if (!nodes[i].has_raw_gnss) {
                continue;
            }
            const Eigen::Vector3d lever_in_odom =
                nodes[i].odom_orientation.toRotationMatrix() * nodes[i].antenna_lever;
            auto *factor = new ceres::AutoDiffCostFunction<RawGnssAnchorFactor, 3, 3, 1>(
                new RawGnssAnchorFactor(nodes[i].raw_gnss, nodes[i].gnss_std, lever_in_odom,
                                        nodes[i].odom_yaw, nodes[i].horizontal_valid,
                                        nodes[i].vertical_valid));
            problem.AddResidualBlock(factor, gnss_loss, positions[i].data(), &yaws[i]);
        }

        // One GNSS position anchor cannot observe global yaw.  Retain the
        // online yaw gauge until multiple anchors provide directional motion.
        if (gnss_anchors < 2) {
            problem.SetParameterBlockConstant(&yaws.front());
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.max_num_iterations = optimize_num_iterations_;
        options.num_threads = 1;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if (!summary.IsSolutionUsable()) {
            return false;
        }

        publishResult(segment_id, nodes, positions, yaws);
        return true;
    }

    void publishResult(int segment_id, const std::vector<RelocNodeData> &nodes,
                       const std::vector<std::array<double, 3>> &positions,
                       const std::vector<double> &yaws) {
        const ros::Time stamp = ros::Time::now();
        nav_msgs::Path combined_path;
        visualization_msgs::MarkerArray segment_markers;
        geometry_msgs::TransformStamped transform;
        std::vector<TrajectoryRow> trajectory_rows;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            SegmentResult result;
            result.segment_id = segment_id;
            result.start_time = nodes.front().time;
            result.end_time = nodes.back().time;

            const size_t last = nodes.size() - 1;
            const double yaw_correction = yaws[last] - nodes[last].odom_yaw;
            result.correction.rotation =
                Eigen::AngleAxisd(yaw_correction, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            result.correction.translation =
                Eigen::Vector3d(positions[last][0], positions[last][1], positions[last][2]) -
                result.correction.rotation * nodes[last].odom_position;

            for (size_t i = 0; i < nodes.size(); i++) {
                result.optimized_poses[nodes[i].node_id] =
                    makeOptimizedPoseStamped(stamp, nodes[i], positions[i], yaws[i]);
            }
            segment_results_[segment_id] = result;
            current_correction_ = result.correction;

            combined_path = buildCombinedPathLocked(stamp);
            trajectory_rows = buildTrajectoryRowsLocked(stamp);
            segment_markers = buildSegmentMarkersLocked(stamp);
            transform = makeMapToOdomTransformLocked(stamp);
        }

        global_path_publisher_.publish(combined_path);
        path_map_publisher_.publish(combined_path);
        reloc_segments_publisher_.publish(segment_markers);
        map_to_odom_publisher_.publish(transform);
        transform_broadcaster_.sendTransform(transform);
        writeGlobalPathFiles(trajectory_rows);
    }

    geometry_msgs::PoseStamped makeOptimizedPoseStamped(
        const ros::Time &stamp, const RelocNodeData &node,
        const std::array<double, 3> &position, double yaw) const {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = "map";
        pose.pose.position.x = position[0];
        pose.pose.position.y = position[1];
        pose.pose.position.z = position[2];

        const double correction_yaw = yaw - node.odom_yaw;
        const Eigen::Quaterniond orientation =
            Eigen::AngleAxisd(correction_yaw, Eigen::Vector3d::UnitZ()) *
            node.odom_orientation;
        pose.pose.orientation.x = orientation.x();
        pose.pose.orientation.y = orientation.y();
        pose.pose.orientation.z = orientation.z();
        pose.pose.orientation.w = orientation.w();
        return pose;
    }

    geometry_msgs::PoseStamped transformNodePoseLocked(
        const ros::Time &stamp, const RelocNodeData &node,
        const MapCorrection &correction) const {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = "map";

        const Eigen::Vector3d position =
            correction.rotation * node.odom_position + correction.translation;
        const Eigen::Quaterniond orientation =
            Eigen::Quaterniond(correction.rotation) * node.odom_orientation;
        pose.pose.position.x = position.x();
        pose.pose.position.y = position.y();
        pose.pose.position.z = position.z();
        pose.pose.orientation.x = orientation.x();
        pose.pose.orientation.y = orientation.y();
        pose.pose.orientation.z = orientation.z();
        pose.pose.orientation.w = orientation.w();
        return pose;
    }

    MapCorrection correctionForTimeLocked(double time) const {
        MapCorrection correction;
        double latest_result_end_time = -std::numeric_limits<double>::infinity();
        for (const auto &result : segment_results_) {
            if (result.second.end_time <= time &&
                result.second.end_time >= latest_result_end_time) {
                correction = result.second.correction;
                latest_result_end_time = result.second.end_time;
            }
        }
        return correction;
    }

    nav_msgs::Path buildCombinedPathLocked(const ros::Time &stamp) const {
        nav_msgs::Path path;
        path.header.stamp = stamp;
        path.header.frame_id = "map";

        for (const auto &node_entry : all_nodes_) {
            const RelocNodeData &node = node_entry.second;
            bool used_optimized_pose = false;
            for (const auto &result : segment_results_) {
                const auto pose = result.second.optimized_poses.find(node.node_id);
                if (pose != result.second.optimized_poses.end()) {
                    geometry_msgs::PoseStamped stamped_pose = pose->second;
                    stamped_pose.header.stamp = stamp;
                    path.poses.push_back(stamped_pose);
                    used_optimized_pose = true;
                    break;
                }
            }
            if (!used_optimized_pose) {
                path.poses.push_back(
                    transformNodePoseLocked(stamp, node, correctionForTimeLocked(node.time)));
            }
        }
        return path;
    }

    std::vector<TrajectoryRow> buildTrajectoryRowsLocked(const ros::Time &stamp) const {
        std::vector<TrajectoryRow> rows;
        rows.reserve(all_nodes_.size());

        for (const auto &node_entry : all_nodes_) {
            const RelocNodeData &node = node_entry.second;
            geometry_msgs::PoseStamped pose;
            bool used_optimized_pose = false;
            for (const auto &result : segment_results_) {
                const auto optimized = result.second.optimized_poses.find(node.node_id);
                if (optimized != result.second.optimized_poses.end()) {
                    pose = optimized->second;
                    used_optimized_pose = true;
                    break;
                }
            }
            if (!used_optimized_pose) {
                pose = transformNodePoseLocked(stamp, node, correctionForTimeLocked(node.time));
            }

            TrajectoryRow row;
            row.gps_time = node.time;
            row.position = Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y,
                                           pose.pose.position.z);
            row.orientation = quaternionFromMessage(pose.pose.orientation);
            rows.push_back(row);
        }

        std::sort(rows.begin(), rows.end(), [](const TrajectoryRow &lhs, const TrajectoryRow &rhs) {
            return lhs.gps_time < rhs.gps_time;
        });
        return rows;
    }

    bool ensureOutputPathLocked() {
        if (output_files_ready_) {
            return true;
        }

        if (outputpath_.empty()) {
            private_node_.getParam("outputpath", outputpath_);
        }
        if (outputpath_.empty()) {
            ros::param::get("/ncf_gvins/outputpath", outputpath_);
        }
        if (outputpath_.empty()) {
            ROS_WARN_THROTTLE(5.0, "NcF-GVINS reloc node waits for /ncf_gvins/outputpath before saving global_path");
            return false;
        }

        global_path_file_ = joinPath(outputpath_, "global_path.csv");
        global_path_unix_file_ = joinPath(outputpath_, "global_path_unix.csv");
        output_files_ready_ = true;
        return true;
    }

    bool updateGpsUnixOffsetLocked() {
        double offset = 0.0;
        if (ros::param::get("/ncf_gvins/gps_unix_offset", offset) && std::isfinite(offset)) {
            gps_unix_offset_ = offset;
            has_gps_unix_offset_ = true;
        }
        return has_gps_unix_offset_;
    }

    void writeTrajectoryFileLocked(const std::string &filename, const std::vector<TrajectoryRow> &rows,
                                   bool use_unix_time) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            ROS_WARN_THROTTLE(5.0, "Failed to open %s for global path output", filename.c_str());
            return;
        }

        file << std::fixed << std::setprecision(9);
        for (const auto &row : rows) {
            const double stamp =
                use_unix_time ? nc_output::unixTimeFromGpsWeekSecond(row.gps_time, gps_unix_offset_)
                              : row.gps_time;
            file << stamp << " " << row.position.x() << " " << row.position.y() << " "
                 << row.position.z() << " " << row.orientation.x() << " " << row.orientation.y()
                 << " " << row.orientation.z() << " " << row.orientation.w() << "\n";
        }
    }

    void writeGlobalPathFiles(const std::vector<TrajectoryRow> &rows) {
        if (rows.empty()) {
            return;
        }

        std::lock_guard<std::mutex> lock(output_mutex_);
        if (!ensureOutputPathLocked()) {
            return;
        }

        writeTrajectoryFileLocked(global_path_file_, rows, false);
        if (updateGpsUnixOffsetLocked()) {
            writeTrajectoryFileLocked(global_path_unix_file_, rows, true);
        }
    }

    visualization_msgs::MarkerArray buildSegmentMarkersLocked(const ros::Time &stamp) const {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker clear;
        clear.header.stamp = stamp;
        clear.header.frame_id = "map";
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear);

        int marker_id = 0;
        for (const auto &result_entry : segment_results_) {
            visualization_msgs::Marker marker;
            marker.header.stamp = stamp;
            marker.header.frame_id = "map";
            marker.ns = "reloc_segments";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.08;
            marker.color.a = 1.0;
            marker.color.r = 0.20 + 0.20 * (marker.id % 3);
            marker.color.g = 0.85 - 0.15 * (marker.id % 2);
            marker.color.b = 1.0 - 0.20 * (marker.id % 4);

            for (const auto &pose_entry : result_entry.second.optimized_poses) {
                geometry_msgs::Point point;
                point.x = pose_entry.second.pose.position.x;
                point.y = pose_entry.second.pose.position.y;
                point.z = pose_entry.second.pose.position.z;
                marker.points.push_back(point);
            }
            if (!marker.points.empty()) {
                markers.markers.push_back(marker);
            }
        }
        return markers;
    }

    geometry_msgs::TransformStamped makeMapToOdomTransformLocked(const ros::Time &stamp) const {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        transform.transform.translation.x = current_correction_.translation.x();
        transform.transform.translation.y = current_correction_.translation.y();
        transform.transform.translation.z = current_correction_.translation.z();
        const Eigen::Quaterniond quaternion(current_correction_.rotation);
        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        transform.transform.rotation.w = quaternion.w();
        return transform;
    }

    void tfTimerCallback(const ros::TimerEvent &) {
        geometry_msgs::TransformStamped transform;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            transform = makeMapToOdomTransformLocked(ros::Time::now());
        }
        map_to_odom_publisher_.publish(transform);
        transform_broadcaster_.sendTransform(transform);
    }

private:
    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    ros::Subscriber frame_subscriber_;
    ros::Subscriber event_subscriber_;
    ros::Publisher global_path_publisher_;
    ros::Publisher path_map_publisher_;
    ros::Publisher map_to_odom_publisher_;
    ros::Publisher reloc_segments_publisher_;
    ros::Timer tf_timer_;
    tf2_ros::TransformBroadcaster transform_broadcaster_;

    std::mutex mutex_;
    std::condition_variable condition_;
    std::thread worker_;
    std::mutex output_mutex_;
    bool finished_{false};
    bool pending_{false};
    nc_reloc::RelocSegmentLifecycle<RelocNodeData> lifecycle_;
    std::map<std::uint64_t, RelocNodeData> all_nodes_;
    std::map<int, SegmentResult> segment_results_;
    MapCorrection current_correction_;

    double relative_position_std_{0.15};
    double relative_yaw_std_deg_{1.0};
    double relative_reference_interval_{0.5};
    int maximum_nodes_{5000};
    int maximum_visualization_nodes_{20000};
    int min_recovery_anchors_{10};
    double max_recovery_tail_duration_{10.0};
    int optimize_num_iterations_{20};
    double tf_publish_rate_{20.0};
    std::string outputpath_;
    std::string global_path_file_;
    std::string global_path_unix_file_;
    bool output_files_ready_{false};
    bool has_gps_unix_offset_{false};
    double gps_unix_offset_{0.0};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "nc_degraded_reloc_node");
    DegradedRelocNode node;
    ros::spin();
    return 0;
}
