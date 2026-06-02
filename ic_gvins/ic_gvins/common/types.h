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

#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Geometry>
#include <cstdint>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// NC-IC extension: the original IC-GVINS accepted or rejected GNSS before it
// reached the estimator.  Keep an explicit health state so degraded samples
// can drive recovery logic without being silently discarded.
enum class SensorHealthState : int {
    UNAVAILABLE = 0,
    ACTIVE      = 1,
    DEGRADED    = 2,
    RECOVERING  = 3,
};

// NC-IC extension: recovery is represented as a segment-level transform from
// unbiased map/NED GNSS observations into the continuous online odom frame.
// Original IC-GVINS has no such split because it uses one global frame only.
typedef struct RecoveryDeviation {
    bool valid{false};
    int segment_id{-1};
    double yaw{0};
    Vector3d translation{0, 0, 0};
    bool yaw_observable{false};
    int supporting_samples{0};
} RecoveryDeviation;

// NC-IC extension: horizontal and vertical GNSS components have independent
// health histories.  This prevents bad height measurements from suppressing
// otherwise useful horizontal positioning.
typedef struct AxisHealth {
    SensorHealthState state{SensorHealthState::UNAVAILABLE};
    int recovery_samples{0};
    double innovation{0};
    bool accepted{false};
    double recovery_start_time{0};
    double last_time{0};
    double covariance_scale{1.0};
} AxisHealth;

// NC-IC extension: modalities other than GNSS use the same health vocabulary
// and weighting contract.  INS remains the propagation backbone; "non-
// centric" means any measurement source can be diagnosed and downweighted.
typedef struct ModalityDecision {
    AxisHealth health;
    bool admit{false};
    double covariance_scale{1.0};
} ModalityDecision;

typedef struct GNSS {
    double time{0};

    // Original IC-GVINS overwrites blh with its local NED measurement.
    // NC-IC keeps that field as the online-factor observation for backwards
    // compatibility, and preserves the unbiased raw/local measurements below.
    Vector3d blh{0, 0, 0};
    Vector3d raw_blh{0, 0, 0};
    Vector3d raw_local{0, 0, 0};
    // NC-IC extension: retained for ROS diagnostics/backwards compatibility;
    // recovery_deviation is the authoritative online observation transform.
    Vector3d online_offset{0, 0, 0};
    Vector3d std{0, 0, 0};

    bool isyawvalid{false};
    double yaw{0};

    // NC-IC extension: quality_valid describes the incoming measurement,
    // forced_degraded is used for outage experiments, and use_online_offset
    // marks GNSS factors adapted to the continuous odom trajectory.
    bool quality_valid{true};
    bool horizontal_valid{true};
    bool vertical_valid{true};
    bool forced_degraded{false};
    bool use_online_offset{false};
    SensorHealthState health_state{SensorHealthState::UNAVAILABLE};
    SensorHealthState horizontal_health_state{SensorHealthState::UNAVAILABLE};
    SensorHealthState vertical_health_state{SensorHealthState::UNAVAILABLE};
    RecoveryDeviation recovery_deviation;

    // NC-IC extension: an optional NED Down measurement bias.  Unlike the
    // recovery transform it models GNSS height error, not odom/map drift.
    bool use_height_bias{false};
    double height_bias{0};
} GNSS;

// NC-IC extension: an estimator-independent packet for the asynchronous
// degraded-segment relocation node.  Online factors may use online_offset,
// while raw_gnss remains the unbiased global/local-NED anchor for map output.
typedef struct RecoveryFrameData {
    double time{0};
    std::uint64_t node_id{0};
    std::uint32_t revision{0};
    int segment_id{-1};
    bool is_keyframe{false};
    Vector3d position{0, 0, 0};
    Quaterniond orientation{1, 0, 0, 0};
    Vector3d antenna_lever{0, 0, 0};

    bool has_raw_gnss{false};
    bool map_only_anchor{false};
    bool horizontal_valid{true};
    bool vertical_valid{true};
    Vector3d raw_gnss{0, 0, 0};
    Vector3d gnss_std{0, 0, 0};
    Vector3d online_offset{0, 0, 0};
    double online_yaw{0};
    bool online_yaw_observable{false};
    double estimated_height_bias{0};
    bool height_bias_valid{false};
    SensorHealthState health_state{SensorHealthState::UNAVAILABLE};
    SensorHealthState horizontal_health_state{SensorHealthState::UNAVAILABLE};
    SensorHealthState vertical_health_state{SensorHealthState::UNAVAILABLE};
} RecoveryFrameData;

enum class RecoveryEventType : int {
    DEGRADED_START     = 0,
    RECOVERY_CONFIRMED = 1,
    SEGMENT_CLOSED     = 2,
    GLOBAL_ALIGNED     = 3,
};

// NC-IC extension: event packets avoid making the asynchronous graph infer
// segment lifecycle from repeatedly updated sliding-window snapshots.
typedef struct RecoveryEventData {
    double time{0};
    int segment_id{-1};
    RecoveryEventType event_type{RecoveryEventType::DEGRADED_START};
    RecoveryDeviation deviation;
} RecoveryEventData;

// NC-IC extension: compact health snapshot for ROS/RViz diagnostics.
typedef struct SensorHealthStatusData {
    double time{0};
    bool nc_extension_enabled{false};
    bool imu_enabled{true};
    bool gnss_enabled{true};
    bool vision_enabled{true};
    bool heading_enabled{false};
    SensorHealthState imu_state{SensorHealthState::UNAVAILABLE};
    SensorHealthState gnss_horizontal_state{SensorHealthState::UNAVAILABLE};
    SensorHealthState gnss_vertical_state{SensorHealthState::UNAVAILABLE};
    SensorHealthState vision_state{SensorHealthState::UNAVAILABLE};
    SensorHealthState heading_state{SensorHealthState::UNAVAILABLE};
    int recovery_segment_id{-1};
    bool recovery_deviation_valid{false};
} SensorHealthStatusData;

// NC-IC extension: heading is a calibrated yaw observation interface.  Raw
// magnetometer calibration is intentionally outside the estimator, allowing
// the current datasets to leave this disabled without changing behaviour.
typedef struct HeadingObservation {
    double time{0};
    double yaw{0};
    double std{0};
    bool valid{false};
    SensorHealthState health_state{SensorHealthState::UNAVAILABLE};
    double innovation{0};
    double covariance_scale{1.0};
} HeadingObservation;

typedef struct VisualQualityReport {
    double time{0};
    int residual_count{0};
    int outlier_count{0};
    double outlier_ratio{0};
    bool valid{false};
    SensorHealthState health_state{SensorHealthState::UNAVAILABLE};
    double covariance_scale{1.0};
} VisualQualityReport;

typedef struct PVA {
    double time;

    Vector3d blh;
    Vector3d vel;
    Vector3d att;
} PVA;

typedef struct IMU {
    double time{0};
    double dt{0};

    Vector3d dtheta{0, 0, 0};
    Vector3d dvel{0, 0, 0};

    double odovel{0};

    // NC-IC extension: bad IMU cannot simply be dropped because it carries
    // propagation time.  Suspect intervals remain propagatable but enlarge
    // their preintegration covariance so they no longer dominate other cues.
    SensorHealthState health_state{SensorHealthState::UNAVAILABLE};
    double noise_scale{1.0};
} IMU;

typedef struct Pose {
    Matrix3d R;
    Vector3d t;
} Pose;

#endif // TYPES_H
