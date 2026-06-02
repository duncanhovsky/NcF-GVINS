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

#ifndef FUSION_ROS_H
#define FUSION_ROS_H

#include "ic_gvins/common/types.h"
#include "ic_gvins/ic_gvins.h"

#include <ic_gvins/RecoveryEvent.h>
#include <ic_gvins/RecoveryFrame.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <queue>
#include <string>

class FusionROS {

public:
    FusionROS() = default;

    void run();

    void setFinished();

private:
    void imuCallback(const sensor_msgs::ImuConstPtr &imumsg);

    void gnssCallback(const sensor_msgs::NavSatFixConstPtr &gnssmsg);

    void imageCallback(const sensor_msgs::ImageConstPtr &imagemsg);

    void publishRecoveryFrame(const RecoveryFrameData &frame);
    void publishRecoveryEvent(const RecoveryEventData &event);
    void publishGnssMeasurement(const GNSS &gnss);
    void publishSensorHealthStatus(const SensorHealthStatusData &status);
    void headingCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &headingmsg);

private:
    std::shared_ptr<GVINS> gvins_;

    IMU imu_{.time = 0}, imu_pre_{.time = 0};
    Frame::Ptr frame_;
    GNSS gnss_;

    bool isusegnssoutage_{false};
    double gnssoutagetime_{0};
    double gnssoutageendtime_{0};
    double gnssthreshold_{20.0};
    bool nc_extension_enabled_{false};
    bool enable_async_relocator_{false};
    bool use_magnetic_heading_{false};
    std::string heading_topic_{"/mag_heading"};

    // NC-IC extension: optimized online snapshots are sent to a separate map
    // correction node; no relocation work runs in the real-time ROS wrapper.
    ros::Publisher recovery_frame_pub_;
    ros::Publisher recovery_event_pub_;
    ros::Publisher gnss_measurement_pub_;
    ros::Publisher sensor_health_panel_pub_;
    sensor_msgs::PointCloud gnss_measurements_;
    ros::WallTime last_sensor_health_panel_time_;
    bool has_sensor_health_panel_time_{false};

    std::queue<IMU> imu_buffer_;
    std::queue<Frame::Ptr> frame_buffer_;
    std::queue<HeadingObservation> heading_buffer_;
};

#endif // FUSION_ROS_H
