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

#include <ceres/ceres.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
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

        frame_subscriber_ =
            node_.subscribe("recovery_frame", 200, &DegradedRelocNode::frameCallback, this);
        event_subscriber_ =
            node_.subscribe("recovery_event", 20, &DegradedRelocNode::eventCallback, this);
        global_path_publisher_ = node_.advertise<nav_msgs::Path>("global_path", 2);
        map_to_odom_publisher_ =
            node_.advertise<geometry_msgs::TransformStamped>("map_to_odom", 2);

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
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto existing = nodes_.find(data.node_id);
            if (existing != nodes_.end() && existing->second.revision > data.revision) {
                return;
            }
            nodes_[data.node_id] = data;
            while (nodes_.size() > static_cast<size_t>(maximum_nodes_)) {
                nodes_.erase(nodes_.begin());
            }
            if (data.segment_id >= 0) {
                auto &segment = segment_nodes_[data.segment_id];
                const auto segment_existing = segment.find(data.node_id);
                if (segment_existing == segment.end() ||
                    segment_existing->second.revision <= data.revision) {
                    segment[data.node_id] = data;
                }
                while (segment.size() > static_cast<size_t>(maximum_nodes_)) {
                    segment.erase(segment.begin());
                }
            }
            // NC-IC extension: node packets now update graph data only.  The
            // event stream explicitly controls recovery lifecycle rather than
            // guessing it from a non-zero online deviation.
            pending_ = relocation_active_;
            should_notify = relocation_active_;
        }
        if (should_notify) {
            condition_.notify_one();
        }
    }

    void eventCallback(const ic_gvins::RecoveryEventConstPtr &message) {
        bool should_notify = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (message->event_type == 0) {
                active_segment_id_ = message->segment_id;
                segment_seen_ = true;
            } else if ((message->event_type == 1 || message->event_type == 3) &&
                       (active_segment_id_ < 0 || message->segment_id == active_segment_id_)) {
                // NC-IC extension: only a confirmed recovery/global alignment
                // allows raw GNSS to correct the archived degraded segment.
                active_segment_id_ = message->segment_id;
                relocation_active_ = true;
                pending_ = true;
            }
            should_notify = relocation_active_;
        }
        if (should_notify) {
            condition_.notify_one();
        }
    }

    void process() {
        while (ros::ok()) {
            std::vector<RelocNodeData> snapshot;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this]() { return pending_; });
                if (finished_) {
                    return;
                }
                pending_ = false;
                const auto active = segment_nodes_.find(active_segment_id_);
                if (active != segment_nodes_.end()) {
                    for (const auto &node : active->second) {
                        snapshot.push_back(node.second);
                    }
                }
            }
            optimizeAndPublish(snapshot);
        }
    }

    void optimizeAndPublish(const std::vector<RelocNodeData> &nodes) {
        if (nodes.size() < 2) {
            return;
        }

        size_t gnss_anchors = 0;
        for (const auto &node : nodes) {
            if (node.has_raw_gnss) {
                gnss_anchors++;
            }
        }
        if (gnss_anchors == 0) {
            return;
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
        options.max_num_iterations = 20;
        options.num_threads = 1;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        publishResult(nodes, positions, yaws);
    }

    void publishResult(const std::vector<RelocNodeData> &nodes,
                       const std::vector<std::array<double, 3>> &positions,
                       const std::vector<double> &yaws) {
        const ros::Time stamp = ros::Time::now();
        nav_msgs::Path path;
        path.header.stamp = stamp;
        path.header.frame_id = "map";

        for (size_t i = 0; i < nodes.size(); i++) {
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = positions[i][0];
            pose.pose.position.y = positions[i][1];
            pose.pose.position.z = positions[i][2];

            const double correction_yaw = yaws[i] - nodes[i].odom_yaw;
            const Eigen::Quaterniond orientation =
                Eigen::AngleAxisd(correction_yaw, Eigen::Vector3d::UnitZ()) * nodes[i].odom_orientation;
            pose.pose.orientation.x = orientation.x();
            pose.pose.orientation.y = orientation.y();
            pose.pose.orientation.z = orientation.z();
            pose.pose.orientation.w = orientation.w();
            path.poses.push_back(pose);
        }
        global_path_publisher_.publish(path);

        const size_t last = nodes.size() - 1;
        const double yaw_correction = yaws[last] - nodes[last].odom_yaw;
        const Eigen::Matrix3d rotation =
            Eigen::AngleAxisd(yaw_correction, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Vector3d translation =
            Eigen::Vector3d(positions[last][0], positions[last][1], positions[last][2]) -
            rotation * nodes[last].odom_position;
        const Eigen::Quaterniond quaternion(rotation);

        geometry_msgs::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        transform.transform.translation.x = translation.x();
        transform.transform.translation.y = translation.y();
        transform.transform.translation.z = translation.z();
        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        transform.transform.rotation.w = quaternion.w();
        map_to_odom_publisher_.publish(transform);
        // NC-IC extension: expose the same correction as TF so consumers can
        // render smooth odom output in the corrected map frame without any
        // reset of IC-GVINS internal states.
        transform_broadcaster_.sendTransform(transform);
    }

private:
    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    ros::Subscriber frame_subscriber_;
    ros::Subscriber event_subscriber_;
    ros::Publisher global_path_publisher_;
    ros::Publisher map_to_odom_publisher_;
    tf2_ros::TransformBroadcaster transform_broadcaster_;

    std::mutex mutex_;
    std::condition_variable condition_;
    std::thread worker_;
    bool finished_{false};
    bool pending_{false};
    bool segment_seen_{false};
    bool relocation_active_{false};
    int active_segment_id_{-1};
    std::map<std::uint64_t, RelocNodeData> nodes_;
    std::map<int, std::map<std::uint64_t, RelocNodeData>> segment_nodes_;

    double relative_position_std_{0.15};
    double relative_yaw_std_deg_{1.0};
    double relative_reference_interval_{0.5};
    int maximum_nodes_{5000};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "nc_degraded_reloc_node");
    DegradedRelocNode node;
    ros::spin();
    return 0;
}
