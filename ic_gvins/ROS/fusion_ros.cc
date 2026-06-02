/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "fusion_ros.h"
#include "drawer_rviz.h"

#include "ic_gvins/common/angle.h"
#include "ic_gvins/common/gpstime.h"
#include "ic_gvins/common/logging.h"
#include "ic_gvins/misc.h"
#include "ic_gvins/tracking/frame.h"

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <memory>
#include <sstream>

std::atomic<bool> isfinished{false};

void sigintHandler(int sig);
void checkStateThread(std::shared_ptr<FusionROS> fusion);

namespace {

const char *healthStateText(SensorHealthState state, bool enabled) {
    if (!enabled) {
        return "DISABLED";
    }
    switch (state) {
    case SensorHealthState::UNAVAILABLE:
        return "UNAVAILABLE";
    case SensorHealthState::ACTIVE:
        return "NORMAL";
    case SensorHealthState::DEGRADED:
        return "DEGRADED";
    case SensorHealthState::RECOVERING:
        return "RECOVERING";
    }
    return "UNKNOWN";
}

void setHealthMarkerColor(visualization_msgs::Marker &marker,
                          SensorHealthState state, bool enabled) {
    marker.color.a = 1.0F;
    if (!enabled || state == SensorHealthState::UNAVAILABLE) {
        marker.color.r = 0.62F;
        marker.color.g = 0.62F;
        marker.color.b = 0.62F;
        return;
    }
    if (state == SensorHealthState::ACTIVE) {
        marker.color.r = 0.18F;
        marker.color.g = 0.95F;
        marker.color.b = 0.28F;
        return;
    }
    if (state == SensorHealthState::DEGRADED) {
        marker.color.r = 1.0F;
        marker.color.g = 0.78F;
        marker.color.b = 0.0F;
        return;
    }
    if (state == SensorHealthState::RECOVERING) {
        marker.color.r = 0.0F;
        marker.color.g = 0.82F;
        marker.color.b = 1.0F;
        return;
    }
    marker.color.r = 1.0F;
    marker.color.g = 0.25F;
    marker.color.b = 0.25F;
}

visualization_msgs::Marker makeHealthTextMarker(
    int id, const std::string &text, double x, double y, double z, double scale) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.ns = "sensor_health_panel";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = scale;
    marker.text = text;
    marker.lifetime = ros::Duration(0.0);
    return marker;
}

} // namespace

void FusionROS::setFinished() {
    if (gvins_ && gvins_->isRunning()) {
        gvins_->setFinished();
    }
}

void FusionROS::run() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // message topic
    string imu_topic, gnss_topic, image_topic, livox_topic;
    pnh.param<string>("imu_topic", imu_topic, "/imu0");
    pnh.param<string>("gnss_topic", gnss_topic, "/gnss0");
    pnh.param<string>("image_topic", image_topic, "/cam0");

    // GVINS parameter
    string configfile;
    pnh.param<string>("configfile", configfile, "gvins.yaml");

    // Load configurations
    YAML::Node config;
    std::vector<double> vecdata;
    try {
        config = YAML::LoadFile(configfile);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return;
    }
    auto outputpath        = config["outputpath"].as<string>();
    auto is_make_outputdir = config["is_make_outputdir"].as<bool>();

    // Create the output directory
    if (!boost::filesystem::is_directory(outputpath)) {
        boost::filesystem::create_directory(outputpath);
    }
    if (!boost::filesystem::is_directory(outputpath)) {
        std::cout << "Failed to open outputpath" << std::endl;
        return;
    }

    if (is_make_outputdir) {
        absl::CivilSecond cs = absl::ToCivilSecond(absl::Now(), absl::LocalTimeZone());
        absl::StrAppendFormat(&outputpath, "/T%04d%02d%02d%02d%02d%02d", cs.year(), cs.month(), cs.day(), cs.hour(),
                              cs.minute(), cs.second());
        boost::filesystem::create_directory(outputpath);
    }

    // GNSS outage configurations
    isusegnssoutage_ = config["isusegnssoutage"].as<bool>();
    gnssoutagetime_  = config["gnssoutagetime"].as<double>();
    gnssoutageendtime_ =
        config["gnssoutageendtime"] ? config["gnssoutageendtime"].as<double>() : 0.0;
    gnssthreshold_   = config["gnssthreshold"].as<double>();

    // NC-IC extension: preserve original filtering unless the new admission
    // layer is explicitly enabled in configuration.
    if (config["nc_extension"]) {
        const auto nc_config = config["nc_extension"];
        nc_extension_enabled_ = nc_config["enabled"] ? nc_config["enabled"].as<bool>() : false;
        enable_async_relocator_ =
            nc_config["enable_async_relocator"] ? nc_config["enable_async_relocator"].as<bool>() : false;
        use_magnetic_heading_ =
            nc_config["use_magnetic_heading"] ? nc_config["use_magnetic_heading"].as<bool>() : false;
        if (nc_config["heading_topic"]) {
            heading_topic_ = nc_config["heading_topic"].as<std::string>();
        }
    }

    // Glog output path
    FLAGS_log_dir = outputpath;

    // The GVINS object
    // NC-IC extension: distinguish the continuous online frame from the
    // asynchronous globally corrected map frame.  Disabled mode preserves the
    // original IC-GVINS visualization frame name.
    Drawer::Ptr drawer =
        std::make_shared<DrawerRviz>(nh, nc_extension_enabled_ ? "odom" : "world");
    gvins_             = std::make_shared<GVINS>(configfile, outputpath, drawer);

    // check is initialized
    if (!gvins_->isRunning()) {
        LOGE << "Fusion ROS terminate";
        return;
    }

    if (nc_extension_enabled_ && enable_async_relocator_) {
        // NC-IC extension: this callback copies optimized state only.  The
        // relocation node owns its graph and uses raw_gnss, so online GNSS
        // offsets cannot bias the corrected global map trajectory.
        recovery_frame_pub_ = nh.advertise<ic_gvins::RecoveryFrame>("recovery_frame", 100);
        // NC-IC extension: lifecycle events are latched so a relocation node
        // started shortly after the online node still receives current segment
        // activation; high-rate frame updates remain unlatched.
        recovery_event_pub_ = nh.advertise<ic_gvins::RecoveryEvent>("recovery_event", 20, true);
        gvins_->setRecoveryFrameCallback(
            [this](const RecoveryFrameData &frame) { publishRecoveryFrame(frame); });
        gvins_->setRecoveryEventCallback(
            [this](const RecoveryEventData &event) { publishRecoveryEvent(event); });
    }
    gnss_measurement_pub_ = nh.advertise<sensor_msgs::PointCloud>("gnss_measurements", 2);
    sensor_health_panel_pub_ =
        nh.advertise<visualization_msgs::MarkerArray>("sensor_health_panel", 2, true);
    gvins_->setGnssMeasurementCallback(
        [this](const GNSS &gnss) { publishGnssMeasurement(gnss); });
    gvins_->setHealthStatusCallback(
        [this](const SensorHealthStatusData &status) { publishSensorHealthStatus(status); });

    // subscribe message
    ros::Subscriber imu_sub   = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200, &FusionROS::imuCallback, this);
    // NC-IC extension: the core now consumes GNSS in timestamp order; retain
    // a modest ROS queue so 10 Hz GNSS is not discarded while optimization is
    // processing an earlier observation.
    ros::Subscriber gnss_sub  = nh.subscribe<sensor_msgs::NavSatFix>(gnss_topic, 50, &FusionROS::gnssCallback, this);
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>(image_topic, 20, &FusionROS::imageCallback, this);
    ros::Subscriber heading_sub;
    if (nc_extension_enabled_ && use_magnetic_heading_) {
        // NC-IC extension: the optional input is a calibrated yaw pose with
        // covariance; raw magnetic-field calibration is deliberately external.
        heading_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            heading_topic_, 20, &FusionROS::headingCallback, this);
    }

    LOGI << "Waiting ROS message; NC=" << nc_extension_enabled_
         << ", async relocation=" << enable_async_relocator_
         << ", calibrated magnetic heading=" << use_magnetic_heading_;

    // enter message loopback
    ros::spin();
}

void FusionROS::imuCallback(const sensor_msgs::ImuConstPtr &imumsg) {
    imu_pre_ = imu_;

    // Time convertion
    double unixsecond = imumsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    imu_.time = weeksec;
    // delta time
    imu_.dt = imu_.time - imu_pre_.time;

    // IMU measurements, Front-Right-Down
    imu_.dtheta[0] = imumsg->angular_velocity.x * imu_.dt;
    imu_.dtheta[1] = imumsg->angular_velocity.y * imu_.dt;
    imu_.dtheta[2] = imumsg->angular_velocity.z * imu_.dt;
    imu_.dvel[0]   = imumsg->linear_acceleration.x * imu_.dt;
    imu_.dvel[1]   = imumsg->linear_acceleration.y * imu_.dt;
    imu_.dvel[2]   = imumsg->linear_acceleration.z * imu_.dt;

    // Not ready
    if (imu_pre_.time == 0) {
        return;
    }

    imu_buffer_.push(imu_);
    while (!imu_buffer_.empty()) {
        auto imu = imu_buffer_.front();

        // Add new IMU to GVINS
        if (gvins_->addNewImu(imu)) {
            imu_buffer_.pop();
        } else {
            // Thread lock failed, try next time
            break;
        }
    }
}

void FusionROS::gnssCallback(const sensor_msgs::NavSatFixConstPtr &gnssmsg) {
    // Time convertion
    double unixsecond = gnssmsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    gnss_ = GNSS();
    gnss_.time = weeksec;

    gnss_.blh[0] = gnssmsg->latitude * D2R;
    gnss_.blh[1] = gnssmsg->longitude * D2R;
    gnss_.blh[2] = gnssmsg->altitude;
    gnss_.std[0] = sqrt(std::max(0.0, gnssmsg->position_covariance[4])); // N
    gnss_.std[1] = sqrt(std::max(0.0, gnssmsg->position_covariance[0])); // E
    gnss_.std[2] = sqrt(std::max(0.0, gnssmsg->position_covariance[8])); // D

    gnss_.isyawvalid = false;

    gnss_.horizontal_valid = std::isfinite(gnss_.std[0]) && std::isfinite(gnss_.std[1]) &&
                             (gnss_.std[0] > 0) && (gnss_.std[1] > 0) &&
                             (gnss_.std[0] < gnssthreshold_) && (gnss_.std[1] < gnssthreshold_);
    gnss_.vertical_valid = std::isfinite(gnss_.std[2]) && (gnss_.std[2] > 0) &&
                           (gnss_.std[2] < gnssthreshold_);
    // NC-IC extension: horizontal availability admits the measurement; the
    // vertical row may independently carry zero information in GnssFactor.
    gnss_.quality_valid = gnss_.horizontal_valid;
    // NC-IC extension: original outage injection never recovered.  An
    // optional end time permits repeatable degraded-then-recovered tests;
    // end <= start retains the original one-sided outage behaviour.
    gnss_.forced_degraded =
        isusegnssoutage_ && (weeksec >= gnssoutagetime_) &&
        ((gnssoutageendtime_ <= gnssoutagetime_) || (weeksec < gnssoutageendtime_));

    if (nc_extension_enabled_) {
        // NC-IC extension: unlike original IC-GVINS, bad/degraded GNSS is
        // forwarded as health evidence.  The core decides factor admission
        // and can recognize the later recovery transition.
        gvins_->addNewGnss(gnss_);
    } else if (gnss_.quality_valid && gnss_.vertical_valid && !gnss_.forced_degraded) {
        // Original IC-GVINS behaviour retained for regression configuration.
        gvins_->addNewGnss(gnss_);
    }
}

void FusionROS::publishGnssMeasurement(const GNSS &gnss) {
    if (!gnss.raw_local.allFinite()) {
        return;
    }
    if (gnss_measurements_.channels.empty()) {
        gnss_measurements_.header.frame_id = "map";
        gnss_measurements_.channels.resize(3);
        gnss_measurements_.channels[0].name = "horizontal_valid";
        gnss_measurements_.channels[1].name = "vertical_valid";
        gnss_measurements_.channels[2].name = "forced_degraded";
    }

    geometry_msgs::Point32 point;
    point.x = static_cast<float>(gnss.raw_local.x());
    point.y = static_cast<float>(gnss.raw_local.y());
    point.z = static_cast<float>(gnss.raw_local.z());

    gnss_measurements_.header.stamp = ros::Time::now();
    gnss_measurements_.points.push_back(point);
    gnss_measurements_.channels[0].values.push_back(gnss.horizontal_valid ? 1.0F : 0.0F);
    gnss_measurements_.channels[1].values.push_back(gnss.vertical_valid ? 1.0F : 0.0F);
    gnss_measurements_.channels[2].values.push_back(gnss.forced_degraded ? 1.0F : 0.0F);

    gnss_measurement_pub_.publish(gnss_measurements_);
}

void FusionROS::publishSensorHealthStatus(const SensorHealthStatusData &status) {
    const ros::WallTime now = ros::WallTime::now();
    if (has_sensor_health_panel_time_ &&
        (now - last_sensor_health_panel_time_).toSec() < 0.2) {
        return;
    }
    last_sensor_health_panel_time_ = now;
    has_sensor_health_panel_time_ = true;

    visualization_msgs::MarkerArray markers;
    const double x = 0.0;
    const double y = -6.0;
    const double z = 6.0;
    const double step = 0.55;

    auto title = makeHealthTextMarker(0, "NcF-GVINS Sensor Health", x, y, z, 0.42);
    title.color.r = 1.0F;
    title.color.g = 1.0F;
    title.color.b = 1.0F;
    title.color.a = 1.0F;
    markers.markers.push_back(title);

    std::ostringstream detail;
    detail << "NC=" << (status.nc_extension_enabled ? "ON" : "OFF")
           << "  t=" << Logging::doubleData(status.time)
           << "  segment=" << status.recovery_segment_id
           << (status.recovery_deviation_valid ? " aligned" : "");
    auto detail_marker = makeHealthTextMarker(1, detail.str(), x, y, z - step, 0.32);
    detail_marker.color.r = 0.80F;
    detail_marker.color.g = 0.84F;
    detail_marker.color.b = 0.90F;
    detail_marker.color.a = 1.0F;
    markers.markers.push_back(detail_marker);

    auto add_status_line = [&](int id, const std::string &name, bool enabled,
                               SensorHealthState state) {
        std::ostringstream text;
        text << name << ": " << healthStateText(state, enabled);
        auto marker = makeHealthTextMarker(id, text.str(), x, y, z - step * id, 0.36);
        setHealthMarkerColor(marker, state, enabled);
        markers.markers.push_back(marker);
    };

    add_status_line(2, "IMU", status.imu_enabled, status.imu_state);
    add_status_line(3, "GNSS horizontal", status.gnss_enabled,
                    status.gnss_horizontal_state);
    add_status_line(4, "GNSS vertical", status.gnss_enabled,
                    status.gnss_vertical_state);
    add_status_line(5, "Vision", status.vision_enabled, status.vision_state);
    add_status_line(6, "Heading", status.heading_enabled, status.heading_state);

    sensor_health_panel_pub_.publish(markers);
}

void FusionROS::publishRecoveryFrame(const RecoveryFrameData &frame) {
    ic_gvins::RecoveryFrame message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "odom";
    message.gps_time = frame.time;
    message.node_id = frame.node_id;
    message.revision = frame.revision;
    message.segment_id = frame.segment_id;
    message.is_keyframe = frame.is_keyframe;

    message.odom_pose.position.x = frame.position.x();
    message.odom_pose.position.y = frame.position.y();
    message.odom_pose.position.z = frame.position.z();
    message.odom_pose.orientation.x = frame.orientation.x();
    message.odom_pose.orientation.y = frame.orientation.y();
    message.odom_pose.orientation.z = frame.orientation.z();
    message.odom_pose.orientation.w = frame.orientation.w();

    message.antenna_lever.x = frame.antenna_lever.x();
    message.antenna_lever.y = frame.antenna_lever.y();
    message.antenna_lever.z = frame.antenna_lever.z();
    message.has_raw_gnss = frame.has_raw_gnss;
    message.map_only_anchor = frame.map_only_anchor;
    message.horizontal_valid = frame.horizontal_valid;
    message.vertical_valid = frame.vertical_valid;
    message.horizontal_health_state = static_cast<int>(frame.horizontal_health_state);
    message.vertical_health_state = static_cast<int>(frame.vertical_health_state);
    message.raw_gnss.x = frame.raw_gnss.x();
    message.raw_gnss.y = frame.raw_gnss.y();
    message.raw_gnss.z = frame.raw_gnss.z();
    message.gnss_std[0] = frame.gnss_std.x();
    message.gnss_std[1] = frame.gnss_std.y();
    message.gnss_std[2] = frame.gnss_std.z();
    message.online_offset.x = frame.online_offset.x();
    message.online_offset.y = frame.online_offset.y();
    message.online_offset.z = frame.online_offset.z();
    message.online_yaw = frame.online_yaw;
    message.online_yaw_observable = frame.online_yaw_observable;
    message.estimated_height_bias = frame.estimated_height_bias;
    message.height_bias_valid = frame.height_bias_valid;
    message.health_state = static_cast<int>(frame.health_state);

    recovery_frame_pub_.publish(message);
}

void FusionROS::publishRecoveryEvent(const RecoveryEventData &event) {
    ic_gvins::RecoveryEvent message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "odom";
    message.gps_time = event.time;
    message.segment_id = event.segment_id;
    message.event_type = static_cast<int>(event.event_type);
    message.deviation_valid = event.deviation.valid;
    message.translation.x = event.deviation.translation.x();
    message.translation.y = event.deviation.translation.y();
    message.translation.z = event.deviation.translation.z();
    message.yaw = event.deviation.yaw;
    message.yaw_observable = event.deviation.yaw_observable;
    message.supporting_samples = event.deviation.supporting_samples;
    recovery_event_pub_.publish(message);
}

void FusionROS::headingCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &headingmsg) {
    HeadingObservation heading;
    double weeksec;
    int week;
    GpsTime::unix2gps(headingmsg->header.stamp.toSec(), week, weeksec);
    heading.time = weeksec;
    const auto &q = headingmsg->pose.pose.orientation;
    heading.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    heading.std = std::sqrt(std::max(0.0, headingmsg->pose.covariance[35]));
    heading.valid = std::isfinite(heading.yaw) && std::isfinite(heading.std) && heading.std > 0;
    if (heading.valid) {
        // NC-IC extension: retain an optional calibrated heading if the
        // optimizer currently owns the state lock, matching other ROS inputs.
        heading_buffer_.push(heading);
        while (!heading_buffer_.empty()) {
            if (gvins_->addNewHeading(heading_buffer_.front())) {
                heading_buffer_.pop();
            } else {
                break;
            }
        }
    }
}

void FusionROS::imageCallback(const sensor_msgs::ImageConstPtr &imagemsg) {
    Mat image;

    // Copy image data
    if (imagemsg->encoding == sensor_msgs::image_encodings::MONO8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);
    } else if (imagemsg->encoding == sensor_msgs::image_encodings::BGR8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
    } else if (imagemsg->encoding == sensor_msgs::image_encodings::RGB8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
        // NC-IC extension: Tracking's grayscale conversion expects BGR
        // storage; preserve RGB images' intended channel weights.
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    }

    // Time convertion
    double unixsecond = imagemsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    // Add new Image to GVINS
    frame_ = Frame::createFrame(weeksec, image);

    frame_buffer_.push(frame_);
    while (!frame_buffer_.empty()) {
        auto frame = frame_buffer_.front();
        if (gvins_->addNewFrame(frame)) {
            frame_buffer_.pop();
        } else {
            break;
        }
    }

    LOG_EVERY_N(INFO, 20) << "Raw data time " << Logging::doubleData(imu_.time) << ", "
                          << Logging::doubleData(gnss_.time) << ", " << Logging::doubleData(frame_->stamp());
}

void sigintHandler(int sig) {
    std::cout << "Terminate by Ctrl+C " << sig << std::endl;
    isfinished = true;
}

void checkStateThread(std::shared_ptr<FusionROS> fusion) {
    std::cout << "Check thread is started..." << std::endl;

    auto fusion_ptr = std::move(fusion);
    while (!isfinished) {
        sleep(1);
    }

    // Exit the GVINS thread
    fusion_ptr->setFinished();

    std::cout << "GVINS has been shutdown ..." << std::endl;

    // Shutdown ROS
    ros::shutdown();

    std::cout << "ROS node has been shutdown ..." << std::endl;
}

int main(int argc, char *argv[]) {
    // Glog initialization
    Logging::initialization(argv, true, true);

    // ROS node
    ros::init(argc, argv, "gvins_node", ros::init_options::NoSigintHandler);

    // Register signal handler
    std::signal(SIGINT, sigintHandler);

    auto fusion = std::make_shared<FusionROS>();

    // Check thread
    std::thread check_thread(checkStateThread, fusion);

    std::cout << "Fusion process is started..." << std::endl;

    // Enter message loop
    fusion->run();

    return 0;
}
