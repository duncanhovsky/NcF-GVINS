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

#include "ic_gvins.h"
#include "misc.h"

#include "common/angle.h"
#include "common/earth.h"
#include "common/gpstime.h"
#include "common/logging.h"

#include "factors/gnss_factor.h"
#include "factors/heading_factor.h"
#include "factors/height_bias_factor.h"
#include "factors/marginalization_factor.h"
#include "factors/marginalization_info.h"
#include "factors/pose_parameterization.h"
#include "factors/recovery_gnss_factor.h"
#include "factors/reprojection_factor.h"
#include "factors/residual_block_info.h"
#include "preintegration/imu_error_factor.h"
#include "preintegration/imu_mix_prior_factor.h"
#include "preintegration/imu_pose_prior_factor.h"
#include "preintegration/preintegration.h"
#include "preintegration/preintegration_factor.h"

#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace {

const char *sensorHealthStateName(SensorHealthState state) {
    switch (state) {
    case SensorHealthState::UNAVAILABLE:
        return "unavailable";
    case SensorHealthState::ACTIVE:
        return "active";
    case SensorHealthState::DEGRADED:
        return "degraded";
    case SensorHealthState::RECOVERING:
        return "recovering";
    }
    return "unknown";
}

bool isRecoverableState(SensorHealthState state) {
    return state == SensorHealthState::DEGRADED || state == SensorHealthState::RECOVERING;
}

std::string joinReasons(const std::vector<std::string> &reasons) {
    if (reasons.empty()) {
        return "condition not specified";
    }
    std::string joined = reasons.front();
    for (size_t i = 1; i < reasons.size(); i++) {
        joined += "; " + reasons[i];
    }
    return joined;
}

void logSensorHealthTransition(const std::string &sensor, SensorHealthState previous,
                               SensorHealthState current, const std::string &degrade_condition,
                               const std::string &recovery_condition, double time) {
    if (current == SensorHealthState::DEGRADED && previous != SensorHealthState::DEGRADED) {
        LOGW << "NC-IC sensor degraded: sensor=" << sensor
             << ", condition=" << degrade_condition
             << ", time=" << Logging::doubleData(time)
             << ", previous=" << sensorHealthStateName(previous);
    } else if (previous == SensorHealthState::DEGRADED &&
               current == SensorHealthState::RECOVERING) {
        LOGW << "NC-IC sensor recovering: sensor=" << sensor
             << ", condition=" << recovery_condition
             << ", time=" << Logging::doubleData(time)
             << ", previous=" << sensorHealthStateName(previous);
    } else if (isRecoverableState(previous) && current == SensorHealthState::ACTIVE) {
        LOGW << "NC-IC sensor recovered: sensor=" << sensor
             << ", condition=" << recovery_condition
             << ", time=" << Logging::doubleData(time)
             << ", previous=" << sensorHealthStateName(previous);
    }
}

std::string gnssInputDegradeReason(const GNSS &gnss, bool horizontal) {
    std::vector<std::string> reasons;
    if (gnss.forced_degraded) {
        reasons.emplace_back("configured GNSS outage injection");
    }
    if (horizontal) {
        if (!gnss.quality_valid) {
            reasons.emplace_back("horizontal quality/covariance invalid");
        }
        if (!gnss.horizontal_valid) {
            reasons.emplace_back("horizontal standard deviation invalid or above gnssthreshold");
        }
    } else if (!gnss.vertical_valid) {
        reasons.emplace_back("vertical standard deviation invalid or above gnssthreshold");
    }
    return joinReasons(reasons);
}

} // namespace

GVINS::GVINS(const string &configfile, const string &outputpath, Drawer::Ptr drawer) {
    gvinsstate_ = GVINS_ERROR;

    // 加载配置
    // Load configuration
    YAML::Node config;
    std::vector<double> vecdata;
    try {
        config = YAML::LoadFile(configfile);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return;
    }

    // 文件IO
    // Output files
    navfilesaver_    = FileSaver::create(outputpath + "/gvins.nav", 11);
    ptsfilesaver_    = FileSaver::create(outputpath + "/mappoint.txt", 3);
    statfilesaver_   = FileSaver::create(outputpath + "/statistics.txt", 3);
    extfilesaver_    = FileSaver::create(outputpath + "/extrinsic.txt", 3);
    imuerrfilesaver_ = FileSaver::create(outputpath + "/IMU_ERR.bin", 7, FileSaver::BINARY);
    trajfilesaver_   = FileSaver::create(outputpath + "/trajectory.csv", 8);

    if (!navfilesaver_->isOpen() || !ptsfilesaver_->isOpen() || !statfilesaver_->isOpen() || !extfilesaver_->isOpen()) {
        LOGE << "Failed to open data file";
        return;
    }

    // Make a copy of configuration file to the output directory
    std::ofstream ofconfig(outputpath + "/gvins.yaml");
    ofconfig << YAML::Dump(config);
    ofconfig.close();

    initlength_       = config["initlength"].as<int>();
    imudatarate_      = config["imudatarate"].as<double>();
    imudatadt_        = 1.0 / imudatarate_;
    reserved_ins_num_ = 2;

    // 安装参数
    // Installation parameters
    vecdata   = config["antlever"].as<std::vector<double>>();
    antlever_ = Vector3d(vecdata.data());

    // IMU噪声参数
    // IMU parameters
    integration_parameters_               = std::make_shared<IntegrationParameters>();
    integration_parameters_->gyr_arw      = config["imumodel"]["arw"].as<double>() * D2R / 60.0;
    integration_parameters_->gyr_bias_std = config["imumodel"]["gbstd"].as<double>() * D2R / 3600.0;
    integration_parameters_->acc_vrw      = config["imumodel"]["vrw"].as<double>() / 60.0;
    integration_parameters_->acc_bias_std = config["imumodel"]["abstd"].as<double>() * 1.0e-5;
    integration_parameters_->corr_time    = config["imumodel"]["corrtime"].as<double>() * 3600;
    integration_parameters_->gravity      = NORMAL_GRAVITY;

    integration_config_.iswithearth = config["iswithearth"].as<bool>();
    configured_with_earth_          = integration_config_.iswithearth;
    integration_config_.isuseodo    = false;
    integration_config_.iswithscale = false;
    integration_config_.gravity     = {0, 0, integration_parameters_->gravity};

    // 初始值, 后续根据GNSS定位实时更新
    // GNSS variables intializaiton
    integration_config_.origin.setZero();
    last_gnss_.blh.setZero();
    gnss_.blh.setZero();

    preintegration_options_ = Preintegration::getOptions(integration_config_);

    // 相机参数
    // Camera parameters
    vector<double> intrinsic  = config["cam0"]["intrinsic"].as<std::vector<double>>();
    vector<double> distortion = config["cam0"]["distortion"].as<std::vector<double>>();
    vector<int> resolution    = config["cam0"]["resolution"].as<std::vector<int>>();

    camera_ = Camera::createCamera(intrinsic, distortion, resolution);

    // IMU和Camera外参
    // Extrinsic parameters
    vecdata           = config["cam0"]["q_b_c"].as<std::vector<double>>();
    Quaterniond q_b_c = Eigen::Quaterniond(vecdata.data());
    vecdata           = config["cam0"]["t_b_c"].as<std::vector<double>>();
    Vector3d t_b_c    = Eigen::Vector3d(vecdata.data());
    td_b_c_           = config["cam0"]["td_b_c"].as<double>();

    pose_b_c_.R = q_b_c.toRotationMatrix();
    pose_b_c_.t = t_b_c;

    // 优化参数
    // Optimization parameters
    reprojection_error_std_      = config["reprojection_error_std"].as<double>();
    optimize_estimate_extrinsic_ = config["optimize_estimate_extrinsic"].as<bool>();
    optimize_estimate_td_        = config["optimize_estimate_td"].as<bool>();
    optimize_num_iterations_     = config["optimize_num_iterations"].as<int>();
    optimize_windows_size_       = config["optimize_windows_size"].as<size_t>();

    // NC-IC extension: all new behaviour is configuration-gated so an
    // unmodified IC-GVINS run remains available as a regression baseline.
    if (config["nc_extension"]) {
        const auto nc_config = config["nc_extension"];
        nc_extension_enabled_ = nc_config["enabled"] ? nc_config["enabled"].as<bool>() : false;

        SensorHealthManager::Options health_options;
        health_options.enabled = nc_extension_enabled_;
        if (nc_config["gnss_recovery_confirm_samples"]) {
            health_options.horizontal_recovery_confirm_samples =
                nc_config["gnss_recovery_confirm_samples"].as<int>();
        }
        if (nc_config["horizontal_recovery_confirm_samples"]) {
            health_options.horizontal_recovery_confirm_samples =
                nc_config["horizontal_recovery_confirm_samples"].as<int>();
        }
        if (nc_config["vertical_recovery_confirm_samples"]) {
            health_options.vertical_recovery_confirm_samples =
                nc_config["vertical_recovery_confirm_samples"].as<int>();
        }
        if (nc_config["horizontal_recovery_confirm_duration"]) {
            health_options.horizontal_recovery_confirm_duration =
                nc_config["horizontal_recovery_confirm_duration"].as<double>();
        }
        if (nc_config["vertical_recovery_confirm_duration"]) {
            health_options.vertical_recovery_confirm_duration =
                nc_config["vertical_recovery_confirm_duration"].as<double>();
        }
        if (nc_config["vision_degrade_confirm_frames"]) {
            health_options.vision_degrade_confirm_frames =
                nc_config["vision_degrade_confirm_frames"].as<int>();
        }
        if (nc_config["vision_recovery_confirm_frames"]) {
            health_options.vision_recovery_confirm_frames =
                nc_config["vision_recovery_confirm_frames"].as<int>();
        }
        if (nc_config["vision_degraded_covariance_scale"]) {
            health_options.vision_degraded_covariance_scale =
                nc_config["vision_degraded_covariance_scale"].as<double>();
        }
        if (nc_config["heading_recovery_confirm_samples"]) {
            health_options.heading_recovery_confirm_samples =
                nc_config["heading_recovery_confirm_samples"].as<int>();
        }
        if (nc_config["heading_degraded_covariance_scale"]) {
            health_options.heading_degraded_covariance_scale =
                nc_config["heading_degraded_covariance_scale"].as<double>();
        }
        if (nc_config["imu_recovery_confirm_intervals"]) {
            health_options.imu_recovery_confirm_intervals =
                nc_config["imu_recovery_confirm_intervals"].as<int>();
        }
        if (nc_config["imu_degraded_covariance_scale"]) {
            health_options.imu_degraded_covariance_scale =
                nc_config["imu_degraded_covariance_scale"].as<double>();
        }
        gnss_health_manager_ = SensorHealthManager(health_options);

        if (nc_config["gnss_timeout"]) {
            gnss_timeout_ = nc_config["gnss_timeout"].as<double>();
        }
        if (nc_config["gnss_horizontal_innovation_threshold"]) {
            gnss_horizontal_innovation_threshold_ =
                nc_config["gnss_horizontal_innovation_threshold"].as<double>();
        }
        if (nc_config["gnss_vertical_innovation_threshold"]) {
            gnss_vertical_innovation_threshold_ =
                nc_config["gnss_vertical_innovation_threshold"].as<double>();
        }
        if (nc_config["recovery_min_horizontal_baseline"]) {
            recovery_min_horizontal_baseline_ =
                nc_config["recovery_min_horizontal_baseline"].as<double>();
        } else if (nc_config["global_alignment_min_baseline"]) {
            recovery_min_horizontal_baseline_ =
                nc_config["global_alignment_min_baseline"].as<double>();
        }
        if (nc_config["recovery_max_yaw_deg"]) {
            recovery_max_yaw_ = nc_config["recovery_max_yaw_deg"].as<double>() * D2R;
        }
        enable_local_bootstrap_ =
            nc_config["enable_local_bootstrap"] ? nc_config["enable_local_bootstrap"].as<bool>() : false;
        if (nc_config["startup_mode"]) {
            force_local_startup_ = nc_config["startup_mode"].as<string>() == "local_static";
        }
        enable_height_bias_ =
            nc_config["enable_height_bias"] ? nc_config["enable_height_bias"].as<bool>() : false;
        if (nc_config["height_bias_random_walk_std"]) {
            height_bias_random_walk_std_ = nc_config["height_bias_random_walk_std"].as<double>();
        }
        if (nc_config["height_bias_prior_std"]) {
            height_bias_prior_std_ = nc_config["height_bias_prior_std"].as<double>();
        }
        use_magnetic_heading_ =
            nc_config["use_magnetic_heading"] ? nc_config["use_magnetic_heading"].as<bool>() : false;
        if (nc_config["heading_max_sync_interval"]) {
            heading_max_sync_interval_ = nc_config["heading_max_sync_interval"].as<double>();
        }
        if (nc_config["heading_innovation_threshold_deg"]) {
            heading_innovation_threshold_ =
                nc_config["heading_innovation_threshold_deg"].as<double>() * D2R;
        }
        if (nc_config["visual_min_residuals"]) {
            visual_min_residuals_ = nc_config["visual_min_residuals"].as<int>();
        }
        if (nc_config["visual_max_outlier_ratio"]) {
            visual_max_outlier_ratio_ = nc_config["visual_max_outlier_ratio"].as<double>();
        }
        if (nc_config["imu_max_angular_rate"]) {
            imu_max_angular_rate_ = nc_config["imu_max_angular_rate"].as<double>();
        }
        if (nc_config["imu_max_specific_force"]) {
            imu_max_specific_force_ = nc_config["imu_max_specific_force"].as<double>();
        }
        if (enable_local_bootstrap_) {
            LocalInitializer::Options local_options;
            local_options.imu_rate = imudatarate_;
            local_options.gravity = integration_parameters_->gravity;
            if (nc_config["local_static_duration"]) {
                local_options.static_duration = nc_config["local_static_duration"].as<double>();
            }
            local_initializer_.reset(new LocalInitializer(local_options));
        }
        if (nc_config["enable_earth_handover"] &&
            nc_config["enable_earth_handover"].as<bool>()) {
            // NC-IC extension: switching a live arbitrary-yaw local window to
            // NED Earth preintegration would require transforming all active
            // poses, landmarks and the marginalization prior.  Keep this
            // unsupported instead of silently mixing incompatible factors.
            LOGW << "NC-IC cannot directly hand over a live local window to Earth mode; "
                    "global alignment remains represented by map_to_odom";
        }
        LOGI << "NC-IC GNSS health/recovery extension is " << (nc_extension_enabled_ ? "enabled" : "disabled");
        LOGI << "NC-IC local bootstrap is " << (enable_local_bootstrap_ ? "enabled" : "disabled")
             << ", height bias is " << (enable_height_bias_ ? "enabled" : "disabled")
             << ", magnetic heading is " << (use_magnetic_heading_ ? "enabled" : "disabled");
    }

    // 归一化相机坐标系下
    // Reprojection std
    optimize_reprojection_error_std_ = reprojection_error_std_ / camera_->focalLength();

    // 可视化
    is_use_visualization_ = config["is_use_visualization"].as<bool>();

    // Initialize the containers
    preintegrationlist_.clear();
    statedatalist_.clear();
    gnsslist_.clear();
    timelist_.clear();

    // GVINS fusion objects
    map_    = std::make_shared<Map>(optimize_windows_size_);
    drawer_ = std::move(drawer);
    drawer_->setMap(map_);
    if (is_use_visualization_) {
        drawer_thread_ = std::thread(&Drawer::run, drawer_);
    }
    tracking_ = std::make_shared<Tracking>(camera_, map_, drawer_, configfile, outputpath);

    // Process threads
    fusion_thread_       = std::thread(&GVINS::runFusion, this);
    tracking_thread_     = std::thread(&GVINS::runTracking, this);
    optimization_thread_ = std::thread(&GVINS::runOptimization, this);

    gvinsstate_ = GVINS_INITIALIZING;
}

void GVINS::setRecoveryFrameCallback(RecoveryFrameCallback callback) {
    std::lock_guard<std::mutex> lock(recovery_callback_mutex_);
    recovery_frame_callback_ = std::move(callback);
}

void GVINS::setRecoveryEventCallback(RecoveryEventCallback callback) {
    std::lock_guard<std::mutex> lock(recovery_callback_mutex_);
    recovery_event_callback_ = std::move(callback);
}

void GVINS::setGnssMeasurementCallback(GnssMeasurementCallback callback) {
    std::lock_guard<std::mutex> lock(gnss_measurement_callback_mutex_);
    gnss_measurement_callback_ = std::move(callback);
}

void GVINS::setHealthStatusCallback(HealthStatusCallback callback) {
    {
        std::lock_guard<std::mutex> lock(health_status_callback_mutex_);
        health_status_callback_ = std::move(callback);
    }
    emitHealthStatus(0.0);
}

bool GVINS::addNewImu(const IMU &input) {
    IMU imu = input;
    if (imu_buffer_mutex_.try_lock()) {
        const bool finite = std::isfinite(imu.time) && std::isfinite(imu.dt) &&
                            imu.dtheta.allFinite() && imu.dvel.allFinite();
        if (!finite || imu.dt <= 0.0) {
            LOGE << "NC-IC rejects non-finite or non-positive-time IMU packet";
            imu_buffer_mutex_.unlock();
            return true;
        }

        const bool has_gap = imu.dt > (imudatadt_ * 1.5);
        const double angular_rate = imu.dtheta.norm() / imu.dt;
        const double specific_force = imu.dvel.norm() / imu.dt;
        const bool amplitude_valid =
            (imu_max_angular_rate_ <= 0.0 || angular_rate <= imu_max_angular_rate_) &&
            (imu_max_specific_force_ <= 0.0 || specific_force <= imu_max_specific_force_);
        SensorHealthState previous_imu_health;
        ModalityDecision decision;
        {
            std::lock_guard<std::mutex> health_lock(health_mutex_);
            previous_imu_health = gnss_health_manager_.imuState();
            decision =
                gnss_health_manager_.updateImu(imu.time, !has_gap && amplitude_valid,
                                                has_gap ? imu.dt : std::max(angular_rate, specific_force));
            imu.health_state = decision.health.state;
            imu.noise_scale = decision.covariance_scale;
        }
        std::vector<std::string> imu_reasons;
        if (has_gap) {
            imu_reasons.emplace_back(
                absl::StrFormat("IMU time gap %.6lf s exceeds %.6lf s", imu.dt, imudatadt_ * 1.5));
        }
        if (!amplitude_valid) {
            if (imu_max_angular_rate_ > 0.0 && angular_rate > imu_max_angular_rate_) {
                imu_reasons.emplace_back(
                    absl::StrFormat("angular rate %.6lf exceeds %.6lf", angular_rate,
                                    imu_max_angular_rate_));
            }
            if (imu_max_specific_force_ > 0.0 && specific_force > imu_max_specific_force_) {
                imu_reasons.emplace_back(
                    absl::StrFormat("specific force %.6lf exceeds %.6lf", specific_force,
                                    imu_max_specific_force_));
            }
        }
        logSensorHealthTransition(
            "IMU", previous_imu_health, decision.health.state, joinReasons(imu_reasons),
            absl::StrFormat("valid IMU intervals accepted by recovery gate, samples=%d",
                            decision.health.recovery_samples),
            imu.time);
        emitHealthStatus(imu.time);
        if (has_gap) {
            LOGW << absl::StrFormat("NC-IC splits suspect IMU gap at %0.3lf dt %0.3lf", imu.time, imu.dt);

            const long cnts = std::max<long>(1, lround(imu.dt / imudatadt_));

            IMU imudata  = imu;
            imudata.time = imu.time - imu.dt;
            imudata.dt = imu.dt / static_cast<double>(cnts);
            imudata.dtheta = imu.dtheta / static_cast<double>(cnts);
            imudata.dvel = imu.dvel / static_cast<double>(cnts);
            for (long k = 0; k < cnts; k++) {
                imudata.time += imudata.dt;
                imu_buffer_.push(imudata);
            }
        } else {
            imu_buffer_.push(imu);
        }

        // 释放信号量
        // Release fusion semaphore
        fusion_sem_.notify_one();

        imu_buffer_mutex_.unlock();
        return true;
    }

    return false;
}

bool GVINS::addNewGnss(const GNSS &gnss) {
    // 低频观测, 无需加锁

    // NC-IC extension: keep the geodetic input untouched.  Original
    // IC-GVINS overwrote its only GNSS value with local NED, which made it
    // impossible for an asynchronous global correction node to distinguish
    // unbiased raw GNSS from an online recovery-adjusted measurement.
    GNSS observation = gnss;
    observation.raw_blh = gnss.blh;

    if (nc_extension_enabled_ && force_local_startup_ && !local_bootstrap_active_) {
        // NC-IC extension: an explicit local-static startup request prevents
        // an early GNSS packet from selecting the original IC initializer
        // before the requested local odom gauge is established.
        return true;
    }

    std::lock_guard<std::mutex> buffer_lock(gnss_buffer_mutex_);

    // 根据GNSS定位更新重力常量
    // Update the gravity from GNSS
    if (integration_config_.origin.isZero()) {
        const bool origin_observation_valid =
            observation.quality_valid && !observation.forced_degraded &&
            (local_bootstrap_active_ || observation.vertical_valid);
        if (nc_extension_enabled_ && !origin_observation_valid) {
            SensorHealthState previous_horizontal;
            SensorHealthState previous_vertical;
            SensorHealthManager::Decision decision;
            {
                std::lock_guard<std::mutex> health_lock(health_mutex_);
                previous_horizontal = gnss_health_manager_.horizontalState();
                previous_vertical = gnss_health_manager_.verticalState();
                decision = gnss_health_manager_.updateGnss(observation.time, false, false);
            }
            const std::string reason =
                "origin initialization requires usable GNSS; " +
                joinReasons({gnssInputDegradeReason(observation, true),
                             gnssInputDegradeReason(observation, false)});
            logSensorHealthTransition("GNSS horizontal", previous_horizontal,
                                      decision.horizontal.state, reason,
                                      "usable horizontal GNSS available for origin initialization",
                                      observation.time);
            logSensorHealthTransition("GNSS vertical", previous_vertical,
                                      decision.vertical.state, reason,
                                      "usable vertical GNSS available for origin initialization",
                                      observation.time);
            emitHealthStatus(observation.time);
            LOGW << "NC-IC waits for GNSS usable by the current startup mode before initializing the origin, condition="
                 << reason;
            return true;
        }

        // 站心原点
        // The origin of the world frame
        integration_config_.origin = observation.raw_blh;
        if (!local_bootstrap_active_) {
            integration_parameters_->gravity = Earth::gravity(observation.raw_blh);
        }
        // NC-IC extension: after local odom is running, later GNSS establishes
        // map coordinates only.  Changing gravity mid-session would make old
        // and new normal preintegrations inconsistent.
        LOGI << "Local gravity is initialized as " << Logging::doubleData(integration_parameters_->gravity);

        if (nc_extension_enabled_ && local_bootstrap_active_) {
            // NC-IC extension: online states already live in arbitrary local
            // odom, so later global GNSS enters through recovery alignment.
            pending_local_global_alignment_ = true;
        }
    }

    observation.raw_local = Earth::global2local(integration_config_.origin, observation.raw_blh);
    observation.blh       = observation.raw_local;

    emitGnssMeasurement(observation);

    if (nc_extension_enabled_ && (gvinsstate_ == GVINS_INITIALIZING) &&
        (!observation.quality_valid || !observation.vertical_valid || observation.forced_degraded)) {
        SensorHealthState previous_horizontal;
        SensorHealthState previous_vertical;
        SensorHealthManager::Decision decision;
        {
            std::lock_guard<std::mutex> health_lock(health_mutex_);
            previous_horizontal = gnss_health_manager_.horizontalState();
            previous_vertical = gnss_health_manager_.verticalState();
            decision = gnss_health_manager_.updateGnss(observation.time, false, false);
        }
        const std::string reason =
            "original IC initialization requires non-degraded GNSS; " +
            joinReasons({gnssInputDegradeReason(observation, true),
                         gnssInputDegradeReason(observation, false)});
        logSensorHealthTransition("GNSS horizontal", previous_horizontal,
                                  decision.horizontal.state, reason,
                                  "valid GNSS available for original IC initialization",
                                  observation.time);
        logSensorHealthTransition("GNSS vertical", previous_vertical,
                                  decision.vertical.state, reason,
                                  "valid GNSS available for original IC initialization",
                                  observation.time);
        emitHealthStatus(observation.time);
        LOGW << "NC-IC rejects degraded GNSS during the original IC initialization stage, condition="
             << reason;
        return true;
    }

    if (!gnss_buffer_.empty() && observation.time <= gnss_buffer_.back().time) {
        LOGW << "NC-IC drops out-of-order GNSS packet at " << Logging::doubleData(observation.time);
        return true;
    }
    // NC-IC extension: buffer GNSS measurements until the fusion thread
    // consumes them in timestamp order; a newer callback can no longer erase
    // recovery evidence or an asynchronous raw anchor.
    gnss_buffer_.push_back(observation);

    return true;
}

bool GVINS::hasPendingGnss(double fusion_time) {
    if (current_gnss_pending_) {
        return true;
    }
    std::lock_guard<std::mutex> lock(gnss_buffer_mutex_);
    return !gnss_buffer_.empty() && gnss_buffer_.front().time < fusion_time;
}

bool GVINS::loadNextGnss(double fusion_time) {
    if (current_gnss_pending_) {
        return true;
    }
    {
        std::lock_guard<std::mutex> lock(gnss_buffer_mutex_);
        if (gnss_buffer_.empty() || gnss_buffer_.front().time >= fusion_time) {
            return false;
        }
        last_last_gnss_ = last_gnss_;
        last_gnss_ = gnss_;
        gnss_ = gnss_buffer_.front();
        gnss_buffer_.pop_front();
    }
    current_gnss_pending_ = true;
    isgnssprepared_ = false;
    last_processed_gnss_time_ = gnss_.time;
    if (pending_local_global_alignment_.exchange(false)) {
        beginRecoverySegment(gnss_.time, "local bootstrap needs first GNSS-to-map alignment");
    }
    return true;
}

void GVINS::consumeCurrentGnss() {
    current_gnss_pending_ = false;
}

void GVINS::checkGnssTimeout(double fusion_time) {
    if (!nc_extension_enabled_ || gnss_timeout_ <= 0.0 || last_processed_gnss_time_ <= 0.0 ||
        gnss_timeout_active_ || (fusion_time - last_processed_gnss_time_) <= gnss_timeout_) {
        return;
    }
    bool is_active = false;
    {
        std::lock_guard<std::mutex> health_lock(health_mutex_);
        is_active = gnss_health_manager_.horizontalState() == SensorHealthState::ACTIVE;
    }
    if (is_active) {
        // NC-IC extension: a configured availability timeout opens a recovery
        // segment when the receiver is silent; it does not estimate data rate.
        beginRecoverySegment(
            fusion_time,
            absl::StrFormat("configured GNSS observation timeout %.3lf s, last GNSS age %.3lf s",
                            gnss_timeout_, fusion_time - last_processed_gnss_time_));
        gnss_timeout_active_ = true;
    }
}

bool GVINS::addNewHeading(const HeadingObservation &heading) {
    if (!use_magnetic_heading_ || !heading.valid || heading.std <= 0) {
        return true;
    }
    latest_heading_ = heading;
    if (gvinsstate_ < GVINS_TRACKING_INITIALIZING) {
        // NC-IC extension: before window factors exist, retain calibrated yaw
        // so a static IC/local initialization may use it as heading evidence.
        return true;
    }

    if (!state_mutex_.try_lock()) {
        return false;
    }
    if (timelist_.empty()) {
        state_mutex_.unlock();
        return true;
    }

    size_t nearest = 0;
    double nearest_dt = std::numeric_limits<double>::max();
    for (size_t k = 0; k < timelist_.size(); k++) {
        const double dt = std::abs(timelist_[k] - heading.time);
        if (dt < nearest_dt) {
            nearest_dt = dt;
            nearest = k;
        }
    }
    if (nearest_dt <= heading_max_sync_interval_) {
        HeadingObservation accepted = heading;
        accepted.time = timelist_[nearest];
        const auto &pose = statedatalist_[nearest].pose;
        const double predicted_yaw =
            std::atan2(2.0 * (pose[6] * pose[5] + pose[3] * pose[4]),
                       1.0 - 2.0 * (pose[4] * pose[4] + pose[5] * pose[5]));
        accepted.innovation =
            std::abs(std::atan2(std::sin(predicted_yaw - accepted.yaw),
                                std::cos(predicted_yaw - accepted.yaw)));
        ModalityDecision decision;
        SensorHealthState previous_heading_health;
        {
            std::lock_guard<std::mutex> health_lock(health_mutex_);
            previous_heading_health = gnss_health_manager_.headingState();
            decision = gnss_health_manager_.updateHeading(
                accepted.time, accepted.innovation <= heading_innovation_threshold_,
                accepted.innovation);
        }
        accepted.health_state = decision.health.state;
        accepted.covariance_scale = decision.covariance_scale;
        logSensorHealthTransition(
            "heading", previous_heading_health, decision.health.state,
            absl::StrFormat("heading innovation %.3lf deg exceeds %.3lf deg",
                            accepted.innovation * R2D,
                            heading_innovation_threshold_ * R2D),
            absl::StrFormat("heading innovation %.3lf deg within %.3lf deg, samples=%d",
                            accepted.innovation * R2D,
                            heading_innovation_threshold_ * R2D,
                            decision.health.recovery_samples),
            accepted.time);
        emitHealthStatus(accepted.time);
        if (!decision.admit) {
            LOGW << "NC-IC rejects calibrated heading by modality health gate, innovation "
                 << accepted.innovation * R2D << " deg";
            state_mutex_.unlock();
            return true;
        }
        accepted.std *= accepted.covariance_scale;
        headinglist_.push_back(accepted);
        isheadingobs_ = true;
        optimization_sem_.notify_one();
        // NC-IC extension: calibrated heading is attached to an existing IC
        // state rather than creating a parallel high-rate state chain.
        LOGI << "NC-IC adds calibrated heading at " << Logging::doubleData(accepted.time);
    }
    state_mutex_.unlock();
    return true;
}

Vector3d GVINS::predictedAntennaPosition() const {
    if (ins_window_.empty()) {
        return Vector3d::Zero();
    }

    IntegrationState state = ins_window_.back().second;
    const size_t state_index = MISC::getInsWindowIndex(ins_window_, gnss_.time);
    if (state_index > 0 && state_index < ins_window_.size()) {
        MISC::statePoseInterpolation(ins_window_[state_index - 1].second,
                                     ins_window_[state_index].second, gnss_.time, state);
    }
    return state.p + state.q.toRotationMatrix() * antlever_;
}

Vector3d GVINS::adjustedOnlineGnssMeasurement(const GNSS &gnss) const {
    if (!gnss.use_online_offset || !gnss.recovery_deviation.valid) {
        return gnss.raw_local;
    }
    const Matrix3d rotation =
        Eigen::AngleAxisd(gnss.recovery_deviation.yaw, Vector3d::UnitZ()).toRotationMatrix();
    return rotation * gnss.raw_local + gnss.recovery_deviation.translation;
}

void GVINS::emitRecoveryEvent(RecoveryEventType type, double time) {
    RecoveryEventCallback callback;
    {
        std::lock_guard<std::mutex> lock(recovery_callback_mutex_);
        callback = recovery_event_callback_;
    }
    if (!callback) {
        return;
    }

    RecoveryEventData event;
    event.time = time;
    event.segment_id = recovery_segment_id_;
    event.event_type = type;
    event.deviation = recovery_deviation_;
    callback(event);
}

void GVINS::emitGnssMeasurement(const GNSS &gnss) {
    GnssMeasurementCallback callback;
    {
        std::lock_guard<std::mutex> lock(gnss_measurement_callback_mutex_);
        callback = gnss_measurement_callback_;
    }
    if (callback) {
        callback(gnss);
    }
}

void GVINS::emitHealthStatus(double time) {
    HealthStatusCallback callback;
    {
        std::lock_guard<std::mutex> lock(health_status_callback_mutex_);
        callback = health_status_callback_;
    }
    if (!callback) {
        return;
    }

    SensorHealthStatusData status;
    status.time = time;
    status.nc_extension_enabled = nc_extension_enabled_;
    status.imu_enabled = true;
    status.gnss_enabled = true;
    status.vision_enabled = true;
    status.heading_enabled = use_magnetic_heading_;
    status.recovery_segment_id = recovery_segment_id_;
    status.recovery_deviation_valid = recovery_deviation_.valid;
    {
        std::lock_guard<std::mutex> health_lock(health_mutex_);
        status.imu_state = gnss_health_manager_.imuState();
        status.gnss_horizontal_state = gnss_health_manager_.horizontalState();
        status.gnss_vertical_state = gnss_health_manager_.verticalState();
        status.vision_state = gnss_health_manager_.visionState();
        status.heading_state = gnss_health_manager_.headingState();
    }
    callback(status);
}

void GVINS::beginRecoverySegment(double time, const string &reason) {
    if (recovery_deviation_.valid) {
        emitRecoveryEvent(RecoveryEventType::SEGMENT_CLOSED, time);
    }
    recovery_segment_id_++;
    recovery_alignment_pairs_.clear();
    recovery_deviation_ = RecoveryDeviation();
    recovery_deviation_.segment_id = recovery_segment_id_;
    {
        std::lock_guard<std::mutex> health_lock(health_mutex_);
        gnss_health_manager_.forceGnssDegraded();
    }
    // NC-IC extension: each lost/recovered episode gets its own transform;
    // drift from an earlier interval must never be silently reused.
    LOGW << "NC-IC sensor degraded: sensor=GNSS, condition=" << reason
         << ", time=" << Logging::doubleData(time)
         << ", recovery_segment=" << recovery_segment_id_;
    emitHealthStatus(time);
    emitRecoveryEvent(RecoveryEventType::DEGRADED_START, time);
}

bool GVINS::estimateRecoveryDeviation() {
    if (recovery_alignment_pairs_.empty()) {
        return false;
    }

    Vector3d mean_map = Vector3d::Zero();
    Vector3d mean_odom = Vector3d::Zero();
    for (const auto &pair : recovery_alignment_pairs_) {
        mean_map += pair.first;
        mean_odom += pair.second;
    }
    mean_map /= static_cast<double>(recovery_alignment_pairs_.size());
    mean_odom /= static_cast<double>(recovery_alignment_pairs_.size());

    double maximum_baseline = 0.0;
    for (size_t i = 0; i < recovery_alignment_pairs_.size(); i++) {
        for (size_t j = i + 1; j < recovery_alignment_pairs_.size(); j++) {
            maximum_baseline =
                std::max(maximum_baseline,
                         (recovery_alignment_pairs_[i].first.head<2>() -
                          recovery_alignment_pairs_[j].first.head<2>()).norm());
        }
    }

    double yaw = 0.0;
    bool yaw_observable = recovery_alignment_pairs_.size() >= 2 &&
                          maximum_baseline >= recovery_min_horizontal_baseline_;
    if (yaw_observable) {
        double cosine_term = 0.0;
        double sine_term = 0.0;
        for (const auto &pair : recovery_alignment_pairs_) {
            const Vector3d map = pair.first - mean_map;
            const Vector3d odom = pair.second - mean_odom;
            cosine_term += map[0] * odom[0] + map[1] * odom[1];
            sine_term += map[0] * odom[1] - map[1] * odom[0];
        }
        yaw = std::atan2(sine_term, cosine_term);
        if (std::abs(yaw) > recovery_max_yaw_) {
            // NC-IC extension: an excessive yaw is treated as unobservable or
            // inconsistent rather than letting a recovery measurement conceal
            // initialization/model errors in the online trajectory.
            LOGW << "NC-IC rejects excessive recovery yaw " << yaw * R2D << " deg";
            yaw = 0.0;
            yaw_observable = false;
        }
    }

    const Matrix3d rotation = Eigen::AngleAxisd(yaw, Vector3d::UnitZ()).toRotationMatrix();
    recovery_deviation_.valid = true;
    recovery_deviation_.yaw = yaw;
    recovery_deviation_.translation = mean_odom - rotation * mean_map;
    recovery_deviation_.yaw_observable = yaw_observable;
    recovery_deviation_.supporting_samples = static_cast<int>(recovery_alignment_pairs_.size());
    return true;
}

bool GVINS::prepareGnssForOnlineFusion() {
    if (isgnssprepared_) {
        return true;
    }

    if (!nc_extension_enabled_) {
        isgnssprepared_ = true;
        return true;
    }

    const bool input_horizontal_valid =
        gnss_.quality_valid && gnss_.horizontal_valid && !gnss_.forced_degraded;
    const bool input_vertical_valid = gnss_.vertical_valid && !gnss_.forced_degraded;
    bool horizontal_valid = input_horizontal_valid;
    bool vertical_valid = input_vertical_valid;
    const bool has_prediction = !ins_window_.empty() && (gvinsstate_ >= GVINS_INITIALIZING_INS);
    SensorHealthState previous_horizontal_health;
    SensorHealthState previous_vertical_health;
    {
        std::lock_guard<std::mutex> health_lock(health_mutex_);
        previous_horizontal_health = gnss_health_manager_.horizontalState();
        previous_vertical_health = gnss_health_manager_.verticalState();
    }
    double horizontal_error = 0.0;
    double vertical_error = 0.0;
    std::vector<std::string> horizontal_reasons;
    std::vector<std::string> vertical_reasons;
    if (!input_horizontal_valid) {
        horizontal_reasons.emplace_back(gnssInputDegradeReason(gnss_, true));
    }
    if (!input_vertical_valid) {
        vertical_reasons.emplace_back(gnssInputDegradeReason(gnss_, false));
    }

    if (horizontal_valid && has_prediction && previous_horizontal_health == SensorHealthState::ACTIVE) {
        // NC-IC extension: test the measurement in the current online frame.
        // After recovery this includes the fixed deviation term; a new large
        // mismatch therefore starts a fresh degraded/recovery episode.
        gnss_.use_online_offset = recovery_deviation_.valid;
        gnss_.recovery_deviation = recovery_deviation_;
        const Vector3d online_measurement = adjustedOnlineGnssMeasurement(gnss_);
        const Vector3d innovation = predictedAntennaPosition() - online_measurement;
        horizontal_error = innovation.head<2>().norm();
        vertical_error = std::abs(innovation[2]);
        if (horizontal_error > gnss_horizontal_innovation_threshold_) {
            horizontal_valid = false;
            horizontal_reasons.emplace_back(
                absl::StrFormat("horizontal innovation %.3lf m exceeds %.3lf m",
                                horizontal_error, gnss_horizontal_innovation_threshold_));
        }
        if (vertical_valid && (vertical_error > gnss_vertical_innovation_threshold_)) {
            // NC-IC extension: a vertical innovation no longer removes good
            // horizontal GNSS; it disables only the height row in the factor.
            vertical_valid = false;
            vertical_reasons.emplace_back(
                absl::StrFormat("vertical innovation %.3lf m exceeds %.3lf m",
                                vertical_error, gnss_vertical_innovation_threshold_));
        }
    }

    SensorHealthManager::Decision decision;
    {
        std::lock_guard<std::mutex> health_lock(health_mutex_);
        decision = gnss_health_manager_.updateGnss(gnss_.time, horizontal_valid, vertical_valid,
                                                    horizontal_error, vertical_error);
    }
    gnss_.health_state = decision.state;
    gnss_.horizontal_health_state = decision.horizontal.state;
    gnss_.vertical_health_state = decision.vertical.state;
    gnss_.horizontal_valid = decision.horizontal.accepted;
    gnss_.vertical_valid = decision.vertical.accepted;
    const std::string horizontal_degrade_reason = joinReasons(horizontal_reasons);
    const std::string vertical_degrade_reason = joinReasons(vertical_reasons);
    const std::string horizontal_recovery_reason =
        absl::StrFormat("valid horizontal GNSS accepted by recovery gate, samples=%d, innovation=%.3lf m",
                        decision.horizontal.recovery_samples, horizontal_error);
    const std::string vertical_recovery_reason =
        absl::StrFormat("valid vertical GNSS accepted by recovery gate, samples=%d, innovation=%.3lf m",
                        decision.vertical.recovery_samples, vertical_error);
    logSensorHealthTransition("GNSS horizontal", previous_horizontal_health,
                              decision.horizontal.state, horizontal_degrade_reason,
                              horizontal_recovery_reason, gnss_.time);
    logSensorHealthTransition("GNSS vertical", previous_vertical_health,
                              decision.vertical.state, vertical_degrade_reason,
                              vertical_recovery_reason, gnss_.time);
    emitHealthStatus(gnss_.time);

    if (decision.state == SensorHealthState::DEGRADED &&
        previous_horizontal_health == SensorHealthState::ACTIVE) {
        beginRecoverySegment(gnss_.time, horizontal_degrade_reason);
    }

    const bool is_recovery_candidate =
        horizontal_valid && has_prediction &&
        (previous_horizontal_health == SensorHealthState::DEGRADED ||
         previous_horizontal_health == SensorHealthState::RECOVERING);
    if (is_recovery_candidate) {
        recovery_alignment_pairs_.emplace_back(gnss_.raw_local, predictedAntennaPosition());
        estimateRecoveryDeviation();
        LOGI << "NC-IC estimates recovery deviation segment " << recovery_segment_id_
             << ", translation " << recovery_deviation_.translation.transpose()
             << ", yaw " << recovery_deviation_.yaw * R2D << " deg";
    }

    if (!decision.accept_online) {
        // NC-IC extension: degraded and unconfirmed recovery observations are
        // preserved as health evidence but do not enter the online window.
        // A geometrically valid recovery candidate still anchors the
        // asynchronous unbiased map, where it cannot jump online odom.
        if (is_recovery_candidate && horizontal_valid) {
            emitRecoveryAnchor(gnss_);
        }
        return false;
    }

    if (recovery_deviation_.valid) {
        // NC-IC extension: the same healthy recovered GNSS is used online
        // through an explicit fixed segment transform. raw_local remains the
        // unchanged globally unbiased anchor for the asynchronous map graph.
        gnss_.recovery_deviation = recovery_deviation_;
        gnss_.online_offset = recovery_deviation_.translation;
        gnss_.use_online_offset = true;
    }
    gnss_.use_height_bias = enable_height_bias_ && gnss_.vertical_valid;
    if (gnss_.use_height_bias) {
        // NC-IC extension: seed a newly accepted height state from the last
        // valid segment value; Ceres then refines it under random walk.
        for (auto it = gnsslist_.rbegin(); it != gnsslist_.rend(); ++it) {
            if (it->use_height_bias) {
                gnss_.height_bias = it->height_bias;
                break;
            }
        }
    }

    if (decision.state == SensorHealthState::ACTIVE &&
        (previous_horizontal_health == SensorHealthState::RECOVERING ||
         previous_horizontal_health == SensorHealthState::DEGRADED) &&
        recovery_deviation_.valid) {
        LOGW << "NC-IC sensor recovered: sensor=GNSS, condition="
             << horizontal_recovery_reason
             << ", time=" << Logging::doubleData(gnss_.time)
             << ", recovery_segment=" << recovery_segment_id_;
        emitRecoveryEvent(RecoveryEventType::RECOVERY_CONFIRMED, gnss_.time);
        gnss_timeout_active_ = false;
        if (local_bootstrap_active_) {
            // NC-IC extension: for a local-start session this recovery is also
            // the first confirmed map-to-odom global alignment.
            emitRecoveryEvent(RecoveryEventType::GLOBAL_ALIGNED, gnss_.time);
        }
    }

    isgnssprepared_ = true;
    return true;
}

void GVINS::emitRecoveryAnchor(const GNSS &gnss) {
    RecoveryFrameCallback callback;
    {
        std::lock_guard<std::mutex> lock(recovery_callback_mutex_);
        callback = recovery_frame_callback_;
    }
    if (!callback || ins_window_.empty()) {
        return;
    }

    IntegrationState state = ins_window_.back().second;
    const size_t state_index = MISC::getInsWindowIndex(ins_window_, gnss.time);
    if (state_index > 0 && state_index < ins_window_.size()) {
        // NC-IC extension: map-only anchors are emitted at the GNSS timestamp.
        // Original IC-GVINS has no asynchronous map graph, so it never needed
        // this extra interpolation boundary.
        MISC::statePoseInterpolation(ins_window_[state_index - 1].second,
                                     ins_window_[state_index].second, gnss.time, state);
    }
    RecoveryFrameData packet;
    packet.time = gnss.time;
    packet.node_id = static_cast<std::uint64_t>(std::llround(gnss.time * 1000000.0));
    packet.revision = ++recovery_frame_revisions_[packet.node_id];
    packet.segment_id = recovery_segment_id_;
    packet.position = state.p;
    packet.orientation = state.q;
    packet.antenna_lever = antlever_;
    packet.has_raw_gnss = true;
    packet.map_only_anchor = true;
    packet.horizontal_valid = gnss.quality_valid && !gnss.forced_degraded;
    packet.vertical_valid = gnss.vertical_valid && !gnss.forced_degraded;
    packet.raw_gnss = gnss.raw_local;
    packet.gnss_std = gnss.std;
    packet.online_offset = recovery_deviation_.translation;
    packet.online_yaw = recovery_deviation_.yaw;
    packet.online_yaw_observable = recovery_deviation_.yaw_observable;
    packet.health_state = gnss.health_state;
    packet.horizontal_health_state = gnss.horizontal_health_state;
    packet.vertical_health_state = gnss.vertical_health_state;
    callback(packet);
}

void GVINS::emitRecoveryFrames() {
    RecoveryFrameCallback callback;
    {
        std::lock_guard<std::mutex> lock(recovery_callback_mutex_);
        callback = recovery_frame_callback_;
    }
    if (!callback) {
        return;
    }

    // NC-IC extension: publish graph-worthy nodes before marginalization.
    // The first implementation copied every window state on every solve;
    // restricting packets to keyframes/GNSS anchors and assigning revisions
    // keeps asynchronous graph traffic bounded while allowing refinements.
    for (const auto &statedata : statedatalist_) {
        RecoveryFrameData packet;
        packet.time = statedata.time;
        packet.node_id = static_cast<std::uint64_t>(std::llround(statedata.time * 1000000.0));
        packet.revision = ++recovery_frame_revisions_[packet.node_id];
        packet.segment_id = recovery_segment_id_;
        packet.position = Vector3d(statedata.pose[0], statedata.pose[1], statedata.pose[2]);
        packet.orientation =
            Quaterniond(statedata.pose[6], statedata.pose[3], statedata.pose[4], statedata.pose[5]);
        packet.antenna_lever = antlever_;
        {
            std::lock_guard<std::mutex> health_lock(health_mutex_);
            packet.health_state = gnss_health_manager_.gnssState();
            packet.horizontal_health_state = gnss_health_manager_.horizontalState();
            packet.vertical_health_state = gnss_health_manager_.verticalState();
        }
        packet.online_offset = recovery_deviation_.translation;
        packet.online_yaw = recovery_deviation_.yaw;
        packet.online_yaw_observable = recovery_deviation_.yaw_observable;

        for (const auto &keyframe : map_->keyframes()) {
            if (MISC::isTheSameTimeNode(keyframe.second->stamp(), statedata.time,
                                        MISC::MINIMUM_TIME_INTERVAL)) {
                packet.is_keyframe = true;
                break;
            }
        }

        for (const auto &gnss : gnsslist_) {
            if (MISC::isTheSameTimeNode(gnss.time, statedata.time, MISC::MINIMUM_TIME_INTERVAL)) {
                packet.has_raw_gnss = true;
                packet.horizontal_valid = gnss.horizontal_valid;
                packet.vertical_valid = gnss.vertical_valid;
                packet.raw_gnss = gnss.raw_local;
                packet.gnss_std = gnss.std;
                packet.online_offset = gnss.online_offset;
                packet.online_yaw = gnss.recovery_deviation.yaw;
                packet.online_yaw_observable = gnss.recovery_deviation.yaw_observable;
                packet.segment_id = gnss.recovery_deviation.segment_id;
                packet.estimated_height_bias = gnss.height_bias;
                packet.height_bias_valid = gnss.use_height_bias;
                packet.health_state = gnss.health_state;
                packet.horizontal_health_state = gnss.horizontal_health_state;
                packet.vertical_health_state = gnss.vertical_health_state;
                break;
            }
        }
        if (packet.is_keyframe || packet.has_raw_gnss) {
            callback(packet);
        }
    }
}

bool GVINS::addNewFrame(const Frame::Ptr &frame) {
    // NC-IC extension: frames are admitted only after either the original
    // GNSS/INS initialization or the explicit static local initializer has
    // provided a metric inertial state.  Raw pre-initialization images alone
    // cannot constitute a valid visual-inertial initialization.
    if (gvinsstate_ > GVINS_INITIALIZING_INS) {
        if (frame_buffer_mutex_.try_lock()) {
            frame_buffer_.push(frame);

            tracking_sem_.notify_one();

            frame_buffer_mutex_.unlock();
            return true;
        }
        return false;
    }
    return true;
}

void GVINS::runFusion() {
    IMU imu_pre, imu_cur;
    IntegrationState state;
    Frame::Ptr frame;

    LOGI << "Fusion thread is started";
    while (!isfinished_) { // While
        Lock lock(fusion_mutex_);
        fusion_sem_.wait(lock);

        // 获取所有有效数据
        // Process all IMU data
        while (true) { // IMU BUFFER
            // 读取IMU缓存
            // Load an IMU sample
            {
                Lock lock2(imu_buffer_mutex_);
                if (imu_buffer_.empty()) {
                    break;
                }
                imu_pre = imu_cur;
                imu_cur = imu_buffer_.front();
                imu_buffer_.pop();
            }

            // INS机械编排及INS处理
            // INS mechanization
            { // INS
                Lock lock3(ins_mutex_);
                if (!ins_window_.empty()) {
                    // 上一时刻的状态
                    // The INS state in last time for mechanization
                    state = ins_window_.back().second;
                }
                ins_window_.emplace_back(imu_cur, IntegrationState());

                // 初始化完成后开始积分输出
                if (gvinsstate_ > GVINS_INITIALIZING) {
                    if (isoptimized_ && state_mutex_.try_lock()) {
                        // 优化求解结束, 需要更新IMU误差重新积分
                        // When the optimization is finished
                        isoptimized_ = false;

                        state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);
                        MISC::redoInsMechanization(integration_config_, state, reserved_ins_num_, ins_window_);

                        state_mutex_.unlock();
                    } else {
                        // 单次机械编排
                        // Do a single INS mechanization
                        MISC::insMechanization(integration_config_, imu_pre, imu_cur, state);

                        ins_window_.back().second = state;
                    }
                } else {
                    // Only reserve certain INS in the window during initialization
                    if (ins_window_.size() > MAXIMUM_INS_NUMBER) {
                        ins_window_.pop_front();
                    }
                }

                // 融合状态
                // Fusion process
                if (gvinsstate_ > GVINS_INITIALIZING && nc_extension_enabled_ &&
                    state_mutex_.try_lock()) {
                    checkGnssTimeout(imu_cur.time);
                    state_mutex_.unlock();
                }
                if (gvinsstate_ == GVINS_INITIALIZING) {
                    if (hasPendingGnss(imu_cur.time) && state_mutex_.try_lock()) {
                        loadNextGnss(imu_cur.time);
                        // 初始化参数
                        // GVINS initialization using GNSS/INS initialization
                        if (gvinsInitialization()) {
                            gvinsstate_ = GVINS_INITIALIZING_INS;

                            // 初始化时需要重新积分
                            // Redo INS mechanization
                            isoptimized_ = true;
                        }
                        consumeCurrentGnss();

                        state_mutex_.unlock();
                        continue;
                    }
                    if (enable_local_bootstrap_ && state_mutex_.try_lock()) {
                        if (gvinsLocalInitialization()) {
                            // NC-IC extension: local initialization supplies
                            // the inertial prior needed by the existing visual
                            // initialization path, without inserting fake GNSS.
                            gvinsstate_ = GVINS_INITIALIZING_VIO;
                            isoptimized_ = true;
                        }
                        state_mutex_.unlock();
                    }
                } else if (gvinsstate_ == GVINS_INITIALIZING_INS) {
                    // 新的GNSS观测到来, 进行优化
                    // New GNSS, do GNSS/INS integration
                    if (hasPendingGnss(imu_cur.time) && state_mutex_.try_lock()) {
                        loadNextGnss(imu_cur.time);
                        // 需要保证数据对齐, 否则等待
                        // For data align
                        if (gnss_.time < ins_window_.back().first.time) {
                            if (prepareGnssForOnlineFusion()) {
                                // 加入新的GNSS节点
                                // Add a new GNSS time node
                                addNewGnssTimeNode();

                                consumeCurrentGnss();
                                isgnssobs_   = true;
                                optimization_sem_.notify_one();
                            } else {
                                // NC-IC extension: retain initialization
                                // states without admitting a degraded global
                                // measurement into the IC window.
                                consumeCurrentGnss();
                            }
                        }
                        state_mutex_.unlock();
                    }
                } else if (gvinsstate_ == GVINS_INITIALIZING_VIO) {
                    // 仅加入关键帧节点, 而不进行优化
                    // Add new time node during the initialization of the visual system
                    if ((isframeready_ || hasPendingGnss(imu_cur.time)) && state_mutex_.try_lock()) {
                        bool frame_time_ready = false;
                        {
                            Lock keyframe_lock(keyframes_mutex_);
                            frame_time_ready = isframeready_ && !keyframes_.empty() &&
                                               (keyframes_.front()->stamp() < ins_window_.back().first.time);
                        }
                        if (frame_time_ready) {
                            addNewKeyFrameTimeNode();

                            isframeready_ = false;

                            gvinsstate_ = GVINS_TRACKING_INITIALIZING;
                        }

                        // 如果有GNSS观测, 也要添加节点
                        // Add GNSS if available
                        if (loadNextGnss(imu_cur.time)) {
                            if (!prepareGnssForOnlineFusion()) {
                                consumeCurrentGnss();
                            } else if (insertNewGnssTimeNode()) {
                                consumeCurrentGnss();
                            }
                        }

                        state_mutex_.unlock();
                    }
                } else if (gvinsstate_ >= GVINS_TRACKING_INITIALIZING) {
                    if ((isframeready_ || hasPendingGnss(imu_cur.time)) && state_mutex_.try_lock()) {
                        bool frame_time_ready = false;
                        {
                            Lock keyframe_lock(keyframes_mutex_);
                            frame_time_ready = isframeready_ && !keyframes_.empty() &&
                                               (keyframes_.front()->stamp() < ins_window_.back().first.time);
                        }
                        if (frame_time_ready) {
                            addNewKeyFrameTimeNode();

                            isframeready_ = false;
                            isvisualobs_  = true;
                        }

                        // 如果有GNSS观测
                        // Add GNSS if available
                        if (loadNextGnss(imu_cur.time)) {
                            if (!prepareGnssForOnlineFusion()) {
                                consumeCurrentGnss();
                            } else if (insertNewGnssTimeNode()) {
                                consumeCurrentGnss();
                                isgnssobs_   = true;
                            }
                        }

                        state_mutex_.unlock();

                        // NC-IC extension: original IC-GVINS only woke this
                        // optimizer for visual observations in normal
                        // tracking.  Recovery GNSS must affect online odom
                        // even when no fresh keyframe arrives at that instant.
                        if (isvisualobs_ || isgnssobs_) {
                            optimization_sem_.notify_one();
                        }
                    }
                }

                // 用于输出
                // For output only
                state = ins_window_.back().second;
            } // INS

            // 总是输出最新的INS机械编排结果, 不占用INS锁
            // Always output the INS results
            if (gvinsstate_ > GVINS_INITIALIZING) {
                MISC::writeNavResult(integration_config_, state, navfilesaver_, imuerrfilesaver_, trajfilesaver_);
            }

        } // IMU BUFFER
    }     // While
}

void GVINS::runOptimization() {

    TimeCost timecost, timecost2;

    LOGI << "Optimization thread is started";
    while (!isfinished_) {
        Lock lock(optimization_mutex_);
        optimization_sem_.wait(lock);

        if (isgnssobs_ || isvisualobs_ || isheadingobs_) {
            timecost.restart();

            // 加锁, 保护状态量
            // Lock the state
            state_mutex_.lock();

            if (gvinsstate_ == GVINS_INITIALIZING_INS) {
                // GINS优化
                // GNSS/INS optimization
                bool isinitialized = gvinsInitializationOptimization();

                if (preintegrationlist_.size() >= static_cast<size_t>(initlength_)) {
                    // 完成GINS初始化, 进入视觉初始化阶段
                    // Enter the initialization of the visual system
                    gvinsstate_ = GVINS_INITIALIZING_VIO;
                    if (isinitialized) {
                        LOGI << "GINS initialization is finished";
                    } else {
                        LOGW << "GINS initialization is not convergence";
                    }
                }
            } else if (gvinsstate_ >= GVINS_TRACKING_INITIALIZING) {

                if (map_->isMaximumKeframes()) {
                    gvinsstate_ = GVINS_TRACKING_NORMAL;
                }

                // 两次非线性优化并进行粗差剔除
                // Two-steps optimization with outlier culling
                gvinsOptimization();

                // NC-IC extension: the asynchronous map node consumes a copy
                // of the optimized online window before old states disappear.
                emitRecoveryFrames();

                timecost2.restart();

                // 移除所有窗口中间插入的非关键帧
                // Remove all non-keyframes time nodes
                gvinsRemoveAllSecondNewFrame();

                // 关键帧数量达到窗口大小, 需要边缘化操作, 并移除最老的关键帧及相关的GNSS和预积分观测, 由于计算力的问题,
                // 可能导致多个关键帧同时加入优化, 需要进行多次边缘化操作
                // Do marginalization
                while (map_->isMaximumKeframes()) {
                    // 边缘化, 移除旧的观测, 按时间对齐到保留的最后一个关键帧
                    gvinsMarginalization();
                }

                timecosts_[2] = timecost2.costInMillisecond();

                // 统计并输出视觉相关的参数
                // Log the statistic parameters
                parametersStatistic();
            }

            // 可视化
            // For visualization
            if (is_use_visualization_) {
                auto state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);
                drawer_->updateMap(MISC::pose2Twc(MISC::stateToCameraPose(state, pose_b_c_)));
            }

            if (isgnssobs_)
                isgnssobs_ = false;
            if (isvisualobs_)
                isvisualobs_ = false;
            if (isheadingobs_)
                isheadingobs_ = false;

            // Release the state lock
            state_mutex_.unlock();
            isoptimized_ = true;

            LOGI << "Optimization costs " << timecost.costInMillisecond() << " ms with " << timecosts_[0] << " and "
                 << timecosts_[1] << " with marginalization costs " << timecosts_[2];
        }
    }
}

void GVINS::runTracking() {
    Frame::Ptr frame;
    Pose pose;
    IntegrationState state, state0, state1;

    std::deque<std::pair<IMU, IntegrationState>> ins_windows;

    LOGI << "Tracking thread is started";
    while (!isfinished_) {
        Lock lock(tracking_mutex_);
        tracking_sem_.wait(lock);

        // 处理所有缓存
        // Process all the frames
        while (true) {
            TimeCost timecost;

            Pose pose_b_c;
            double td;
            {
                Lock lock3(extrinsic_mutex_);
                pose_b_c = pose_b_c_;
                td       = td_b_c_;
            }

            // 读取缓存
            {
                frame_buffer_mutex_.lock();
                if (frame_buffer_.empty()) {
                    frame_buffer_mutex_.unlock();
                    break;
                }
                frame = frame_buffer_.front();

                // 保证每个图像都有先验的惯导位姿
                // Wait until the INS is available
                ins_mutex_.lock();
                if (ins_window_.empty() || (ins_window_.back().second.time <= (frame->stamp() + td))) {
                    ins_mutex_.unlock();
                    frame_buffer_mutex_.unlock();

                    usleep(1000);
                    continue;
                }
                ins_mutex_.unlock();

                frame_buffer_.pop();
                frame_buffer_mutex_.unlock();
            }

            // 获取初始位姿
            // The prior pose from INS
            {
                Lock lock2(ins_mutex_);
                frame->setStamp(frame->stamp() + td);
                frame->setTimeDelay(td);
                MISC::getCameraPoseFromInsWindow(ins_window_, pose_b_c, frame->stamp(), pose);
                frame->setPose(pose);
            }

            TrackState trackstate = tracking_->track(frame);
            if (trackstate == TRACK_LOST) {
                LOGE << "Tracking lost at " << Logging::doubleData(frame->stamp());
            }

            // 包括第一帧在内的所有关键帧, 跟踪失败时的当前帧也会成为新的关键帧
            // All possible keyframes
            if (tracking_->isNewKeyFrame() || (trackstate == TRACK_FIRST_FRAME) || trackstate == TRACK_LOST) {
                Lock lock3(keyframes_mutex_);
                keyframes_.push(frame);

                isframeready_ = true;

                LOGI << "Tracking cost " << timecost.costInMillisecond() << " ms";
            }
        }
    }
}

void GVINS::setFinished() {
    isfinished_ = true;

    // 释放信号量, 退出所有线程
    // Release all semaphores
    fusion_sem_.notify_all();
    tracking_sem_.notify_all();
    optimization_sem_.notify_all();

    tracking_thread_.join();
    optimization_thread_.join();
    fusion_thread_.join();

    if (is_use_visualization_) {
        drawer_->setFinished();
        drawer_thread_.join();
    }

    Quaterniond q_b_c = Rotation::matrix2quaternion(pose_b_c_.R);
    Vector3d t_b_c    = pose_b_c_.t;

    LOGW << "GVINS has finished processing";
    LOGW << "Estimated extrinsics: "
         << absl::StrFormat("(%0.6lf, %0.6lf, %0.6lf, %0.6lf), (%0.3lf, %0.3lf, "
                            "%0.3lf), %0.4lf",
                            q_b_c.x(), q_b_c.y(), q_b_c.z(), q_b_c.w(), t_b_c.x(), t_b_c.y(), t_b_c.z(), td_b_c_);

    Logging::shutdownLogging();
}

bool GVINS::gvinsInitialization() {

    if ((gnss_.time == 0) || (last_gnss_.time == 0)) {
        return false;
    }

    // 缓存数据用于零速检测
    // Buffer for zero-velocity detection
    vector<IMU> imu_buff;
    for (const auto &ins : ins_window_) {
        auto &imu = ins.first;
        if ((imu.time > last_gnss_.time) && (imu.time < gnss_.time)) {
            imu_buff.push_back(imu);
        }
    }
    if (imu_buff.size() < 20) {
        return false;
    }

    // 零速检测估计陀螺零偏和横滚俯仰角
    // Obtain the gyroscope biases and roll and pitch angles
    vector<double> average;
    static Vector3d bg{0, 0, 0};
    static Vector3d initatt{0, 0, 0};
    static bool is_has_zero_velocity = false;

    bool is_zero_velocity = MISC::detectZeroVelocity(imu_buff, imudatarate_, average);
    if (is_zero_velocity) {
        // 陀螺零偏
        bg = Vector3d(average[0], average[1], average[2]);
        bg *= imudatarate_;

        // 重力调平获取横滚俯仰角
        Vector3d fb(average[3], average[4], average[5]);
        fb *= imudatarate_;

        initatt[0] = -asin(fb[1] / integration_parameters_->gravity);
        initatt[1] = asin(fb[0] / integration_parameters_->gravity);

        LOGI << "Zero velocity get gyroscope bias " << bg.transpose() * 3600 * R2D << ", roll " << initatt[0] * R2D
             << ", pitch " << initatt[1] * R2D;
        is_has_zero_velocity = true;
    }

    // 非零速状态
    // Initialization conditions
    if (!is_zero_velocity) {
        if (last_gnss_.isyawvalid) {
            initatt[2] = last_gnss_.yaw;
            LOGI << "Initialized heading from dual-antenna GNSS as " << initatt[2] * R2D << " deg";
        } else {
            const double gnss_dt = gnss_.time - last_gnss_.time;
            if (gnss_dt <= 0.0) {
                return false;
            }
            // NC-IC extension: the original threshold was applied to
            // displacement between adjacent GNSS packets, making startup
            // depend on GNSS rate.  Apply the configured motion criterion to
            // velocity so 1 Hz and 10 Hz receivers express the same rule.
            Vector3d vel = (gnss_.blh - last_gnss_.blh) / gnss_dt;
            if (vel.norm() < MINMUM_ALIGN_VELOCITY) {
                return false;
            }

            if (!is_has_zero_velocity) {
                initatt[0] = 0;
                initatt[1] = atan(-vel.z() / sqrt(vel.x() * vel.x() + vel.y() * vel.y()));
                LOGI << "Initialized pitch from GNSS as " << initatt[1] * R2D << " deg";
            }
            initatt[2] = atan2(vel.y(), vel.x());
            LOGI << "Initialized heading from GNSS as " << initatt[2] * R2D << " deg";
        }
    } else if (use_magnetic_heading_ && latest_heading_.valid) {
        // NC-IC extension: original IC waits for GNSS motion to initialize
        // heading.  A configured calibrated heading can complete a static
        // initialization, while default-off datasets retain original logic.
        initatt[2] = latest_heading_.yaw;
        LOGI << "Initialized heading from calibrated external heading as "
             << initatt[2] * R2D << " deg";
    } else {
        return false;
    }

    // 从零速开始
    Vector3d velocity = Vector3d::Zero();

    // 初始状态, 从上一秒开始
    // The initialization state
    auto state = IntegrationState{
        .time = last_gnss_.time,
        .p    = last_gnss_.blh - Rotation::euler2quaternion(initatt) * antlever_,
        .q    = Rotation::euler2quaternion(initatt),
        .v    = velocity,
        .bg   = bg,
        .ba   = {0, 0, 0},
        .sodo = 0.0,
        .sg   = {0, 0, 0},
        .sa   = {0, 0, 0},
    };
    statedatalist_.emplace_back(Preintegration::stateToData(state, preintegration_options_));
    gnsslist_.push_back(last_gnss_);
    timelist_.push_back(last_gnss_.time);
    constructPrior(is_has_zero_velocity);

    // 初始化重力和地球自转参数
    // The gravity and the Earth rotation rate
    integration_config_.gravity = Vector3d(0, 0, integration_parameters_->gravity);
    if (integration_config_.iswithearth) {
        integration_config_.iewn = Earth::iewn(integration_config_.origin, state.p);
    }

    // 计算第一秒的INS结果
    // Redo INS mechanization at the first second
    state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);
    MISC::redoInsMechanization(integration_config_, state, reserved_ins_num_, ins_window_);

    LOGI << "Initialization at " << Logging::doubleData(gnss_.time);

    if (nc_extension_enabled_) {
        SensorHealthState previous_horizontal_health;
        SensorHealthState previous_vertical_health;
        SensorHealthManager::Decision decision;
        // NC-IC extension: GNSS accepted for original IC initialization is
        // already trusted. Seed health so a following silent outage is
        // observable without waiting for another positioning packet.
        {
            std::lock_guard<std::mutex> health_lock(health_mutex_);
            previous_horizontal_health = gnss_health_manager_.horizontalState();
            previous_vertical_health = gnss_health_manager_.verticalState();
            decision = gnss_health_manager_.updateGnss(gnss_.time, true, true);
        }
        logSensorHealthTransition(
            "GNSS horizontal", previous_horizontal_health, decision.horizontal.state,
            "GNSS was not usable before initialization",
            "GNSS accepted for original IC initialization", gnss_.time);
        logSensorHealthTransition(
            "GNSS vertical", previous_vertical_health, decision.vertical.state,
            "GNSS height was not usable before initialization",
            "GNSS height accepted for original IC initialization", gnss_.time);
        emitHealthStatus(gnss_.time);
    }

    // 加入当前GNSS时间节点
    // Add current GNSS time node
    addNewGnssTimeNode();

    return true;
}

bool GVINS::gvinsLocalInitialization() {
    if (!nc_extension_enabled_ || !enable_local_bootstrap_ || !local_initializer_ ||
        local_bootstrap_active_) {
        return false;
    }
    if (!force_local_startup_ && !integration_config_.origin.isZero()) {
        // NC-IC extension: if a trustworthy GNSS origin has already arrived,
        // preserve the original IC Earth-aware startup instead of silently
        // falling back to the less informative local mode.
        return false;
    }

    IntegrationState state;
    const HeadingObservation *heading =
        (use_magnetic_heading_ && latest_heading_.valid) ? &latest_heading_ : nullptr;
    if (!local_initializer_->tryInitialize(ins_window_, heading, state)) {
        return false;
    }

    // NC-IC extension: without geographic origin there is no valid Earth-rate
    // expression in NED.  This local-odom session intentionally uses normal
    // preintegration; the original trusted-GNSS startup retains Earth mode.
    integration_config_.iswithearth = false;
    integration_config_.islocalframe = true;
    integration_config_.gravity = Vector3d(0, 0, integration_parameters_->gravity);
    preintegration_options_ = Preintegration::getOptions(integration_config_);

    statedatalist_.emplace_back(Preintegration::stateToData(state, preintegration_options_));
    timelist_.push_back(state.time);
    constructPrior(true);
    local_bootstrap_active_ = true;

    LOGW << "NC-IC initialized local odom without GNSS at " << Logging::doubleData(state.time)
         << "; Earth-aware online propagation is retained only by the original GNSS-start path";
    return true;
}

bool GVINS::gvinsInitializationOptimization() {
    // GNSS/INS optimization

    // 构建优化问题
    ceres::Solver solver;
    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations         = 50;

    // 参数块
    // Add parameter blocks
    addStateParameters(problem);

    // GNSS残差
    // Add gnss factors
    addGnssFactors(problem, true);

    // 预积分残差
    // Add IMU preintegration factors
    addImuFactors(problem);

    solver.Solve(options, &problem, &summary);
    LOGI << summary.BriefReport();

    return summary.termination_type == ceres::CONVERGENCE;
}

void GVINS::addNewKeyFrameTimeNode() {

    Lock lock(keyframes_mutex_);
    vector<double> extra_gnss_node;
    while (!keyframes_.empty()) {
        // 取出一个关键帧
        // Obtain a new valid keyframe
        auto frame       = keyframes_.front();
        double frametime = frame->stamp();
        if (frametime > ins_window_.back().first.time) {
            break;
        }

        keyframes_.pop();

        // 添加关键帧
        // Add new keyframe time node
        LOGI << "Insert keyframe " << frame->keyFrameId() << " at " << Logging::doubleData(frame->stamp()) << " with "
             << frame->unupdatedMappoints().size() << " new mappoints";
        map_->insertKeyFrame(frame);

        addNewTimeNode(frametime);
        LOGI << "Add new keyframe time node at " << Logging::doubleData(frametime);
    }

    // 移除多余的预积分节点
    // Remove unused time node
    removeUnusedTimeNode();
}

bool GVINS::removeUnusedTimeNode() {
    if (unused_time_nodes_.empty()) {
        return false;
    }

    LOGI << "Remove " << unused_time_nodes_.size() << " unused time node "
         << Logging::doubleData(unused_time_nodes_[0]);

    for (double node : unused_time_nodes_) {
        int index = getStateDataIndex(node);

        // Exception
        if (index < 0) {
            continue;
        }

        auto first_preintegration  = preintegrationlist_[index - 1];
        auto second_preintegration = preintegrationlist_[index];
        auto imu_buffer            = second_preintegration->imuBuffer();

        // 将后一个预积分的IMU数据合并到前一个, 不包括第一个IMU数据
        // Merge the IMU preintegration
        for (size_t k = 1; k < imu_buffer.size(); k++) {
            first_preintegration->addNewImu(imu_buffer[k]);
        }

        // 移除时间节点, 以及后一个预积分
        // Remove the second time node
        preintegrationlist_.erase(preintegrationlist_.begin() + index);
        timelist_.erase(timelist_.begin() + index);
        statedatalist_.erase(statedatalist_.begin() + index);
    }
    unused_time_nodes_.clear();

    return true;
}

bool GVINS::insertNewGnssTimeNode() {
    // Wait new keyframe to determine GNSS time ndoe
    if (gnss_.time > timelist_.back()) {
        return false;
    }

    // Find time interval
    double sta = 0, end = 0;
    size_t index = 0;
    for (size_t k = timelist_.size() - 1; k > 1; k--) {
        if ((gnss_.time <= timelist_[k]) && (gnss_.time > timelist_[k - 1])) {
            sta   = timelist_[k - 1];
            end   = timelist_[k];
            index = k;
        }
    }
    if (sta == 0) {
        return false;
    }

    // Check the keyframe is a normal keyframe
    bool is_need_gnss = false;
    auto keyframeids  = map_->orderedKeyFrames();
    for (int k = keyframeids.size() - 1; k >= 0; k--) {
        auto frame = map_->keyframes().find(keyframeids[k])->second;

        double keyframe_time = frame->stamp();
        if (MISC::isTheSameTimeNode(keyframe_time, end, MISC::MINIMUM_TIME_INTERVAL)) {
            if (frame->keyFrameState() != KEYFRAME_REMOVE_SECOND_NEW) {
                is_need_gnss = true;
            }
        }
    }
    if (!is_need_gnss) {
        LOGI << "Unused GNSS due to non-normal keyframe at " << Logging::doubleData(gnss_.time);
        return true;
    }

    if (gnss_.time - sta < MINMUM_SYNC_INTERVAL) {
        // Align to previous node
        GNSS gnss = gnss_;
        gnss.time = sta;

        // Compensate the time
        double dt = gnss_.time - sta;
        gnss.blh[0] -= statedatalist_[index - 1].mix[0] * dt;
        gnss.blh[1] -= statedatalist_[index - 1].mix[1] * dt;
        gnss.blh[2] -= statedatalist_[index - 1].mix[2] * dt;
        // NC-IC extension: keep the raw anchor synchronized to the same time
        // node without applying online_offset; the asynchronous map factor
        // must remain unbiased but temporally aligned.
        gnss.raw_local[0] -= statedatalist_[index - 1].mix[0] * dt;
        gnss.raw_local[1] -= statedatalist_[index - 1].mix[1] * dt;
        gnss.raw_local[2] -= statedatalist_[index - 1].mix[2] * dt;
        gnss.std *= 1.2;

        gnsslist_.push_back(gnss);
        LOGI << "Add new GNSS " << Logging::doubleData(gnss_.time) << " align to " << Logging::doubleData(sta);
    } else if (end - gnss_.time < MINMUM_SYNC_INTERVAL) {
        // Align to current node
        GNSS gnss = gnss_;
        gnss.time = end;

        // Compensate the time
        double dt = end - gnss_.time;
        gnss.blh[0] += statedatalist_[index].mix[0] * dt;
        gnss.blh[1] += statedatalist_[index].mix[1] * dt;
        gnss.blh[2] += statedatalist_[index].mix[2] * dt;
        gnss.raw_local[0] += statedatalist_[index].mix[0] * dt;
        gnss.raw_local[1] += statedatalist_[index].mix[1] * dt;
        gnss.raw_local[2] += statedatalist_[index].mix[2] * dt;
        gnss.std *= 1.2;

        gnsslist_.push_back(gnss);
        LOGI << "Add new GNSS " << Logging::doubleData(gnss_.time) << " align to " << Logging::doubleData(end);
    } else {
        // Avoid reintegrating the long-time preintegration
        if (preintegrationlist_[index - 1]->deltaTime() > MAXIMUM_PREINTEGRATION_LENGTH) {
            LOGI << "Unused GNSS due to long-time preintegration " << Logging::doubleData(gnss_.time);
            return true;
        }

        // Insert GNSS node to sliding window
        vector<double> timelist;
        for (size_t k = index; k < timelist_.size(); k++) {
            timelist.push_back(timelist_[k]);
        }

        // Remove back time node
        size_t num_remove = timelist_.size() - index;
        for (size_t k = num_remove; k > 0; k--) {
            timelist_.pop_back();
            statedatalist_.pop_back();
            preintegrationlist_.pop_back();
        }

        // Add GNSS time node
        addNewGnssTimeNode();

        // Add back time node
        for (size_t k = 0; k < timelist.size(); k++) {
            addNewTimeNode(timelist[k]);
        }
    }

    return true;
}

void GVINS::addNewGnssTimeNode() {
    LOGI << "Add new GNSS time node " << Logging::doubleData(gnss_.time);

    addNewTimeNode(gnss_.time);
    gnsslist_.push_back(gnss_);
}

void GVINS::addNewTimeNode(double time) {

    vector<IMU> series;
    IntegrationState state;

    // 获取时段内用于预积分的IMU数据
    // Obtain the IMU samples between the two time nodes
    double start = timelist_.back();
    double end   = time;
    MISC::getImuSeriesFromTo(ins_window_, start, end, series);

    state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);

    // 新建立新的预积分
    // Build a new IMU preintegration
    preintegrationlist_.emplace_back(
        Preintegration::createPreintegration(integration_parameters_, series[0], state, preintegration_options_));

    // 预积分, 从第二个历元开始
    // Add IMU sample
    for (size_t k = 1; k < series.size(); k++) {
        preintegrationlist_.back()->addNewImu(series[k]);
    }

    // 当前状态加入到滑窗中
    // Add current state and time node to the sliding window
    state      = preintegrationlist_.back()->currentState();
    state.time = time;

    statedatalist_.emplace_back(Preintegration::stateToData(state, preintegration_options_));
    timelist_.push_back(time);
}

void GVINS::parametersStatistic() {

    vector<double> parameters;

    // 所有关键帧
    // All keyframes
    vector<ulong> keyframeids = map_->orderedKeyFrames();
    size_t size               = keyframeids.size();
    if (size < 2) {
        return;
    }
    auto keyframes = map_->keyframes();

    // 最新的关键帧
    // The latest keyframe
    auto frame_cur = keyframes.at(keyframeids[size - 1]);
    auto frame_pre = keyframes.at(keyframeids[size - 2]);

    // 时间戳
    // Time stamp
    parameters.push_back(frame_cur->stamp());
    parameters.push_back(frame_cur->stamp() - frame_pre->stamp());

    // 当前关键帧与上一个关键帧的id差, 即最新关键帧的跟踪帧数
    // Interval
    auto frame_cnt = static_cast<double>(frame_cur->id() - frame_pre->id());
    parameters.push_back(frame_cnt);

    // 特征点数量
    // Feature points
    parameters.push_back(static_cast<double>(frame_cur->numFeatures()));

    // 路标点重投影误差统计
    // Reprojection errors
    vector<double> reprojection_errors;
    for (auto &landmark : map_->landmarks()) {
        auto mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        vector<double> errors;
        for (auto &observation : mappoint->observations()) {
            auto feat = observation.lock();
            if (!feat || feat->isOutlier()) {
                continue;
            }
            auto frame = feat->getFrame();
            if (!frame || !frame->isKeyFrame() || !map_->isKeyFrameInMap(frame)) {
                continue;
            }

            double error = camera_->reprojectionError(frame->pose(), mappoint->pos(), feat->keyPoint()).norm();
            errors.push_back(error);
        }
        if (errors.empty()) {
            LOGE << "Mappoint " << mappoint->id() << " with zero observation";
            continue;
        }
        double avg_error = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
        reprojection_errors.emplace_back(avg_error);
    }

    if (reprojection_errors.empty()) {
        reprojection_errors.push_back(0);
    }

    double min_error = *std::min_element(reprojection_errors.begin(), reprojection_errors.end());
    parameters.push_back(min_error);
    double max_error = *std::max_element(reprojection_errors.begin(), reprojection_errors.end());
    parameters.push_back(max_error);
    double avg_error = std::accumulate(reprojection_errors.begin(), reprojection_errors.end(), 0.0) /
                       static_cast<double>(reprojection_errors.size());
    parameters.push_back(avg_error);
    double sq_sum =
        std::inner_product(reprojection_errors.begin(), reprojection_errors.end(), reprojection_errors.begin(), 0.0);
    double rms_error = std::sqrt(sq_sum / static_cast<double>(reprojection_errors.size()));
    parameters.push_back(rms_error);

    // 迭代次数
    // Iterations
    parameters.push_back(iterations_[0]);
    parameters.push_back(iterations_[1]);

    // 计算耗时
    // Time cost
    parameters.push_back(timecosts_[0]);
    parameters.push_back(timecosts_[1]);
    parameters.push_back(timecosts_[2]);

    // 路标点粗差
    // Outliers
    parameters.push_back(outliers_[0]);
    parameters.push_back(outliers_[1]);

    // 保存数据
    // Dump current parameters
    statfilesaver_->dump(parameters);
    statfilesaver_->flush();
}

bool GVINS::gvinsOutlierCulling() {
    if (map_->keyframes().empty()) {
        return false;
    }

    // 移除非关键帧中的路标点, 不能在遍历中直接移除, 否则破坏了遍历
    // Find outliers first and remove later
    vector<MapPoint::Ptr> mappoints;
    int num_outliers_mappoint = 0;
    int num_outliers_feature  = 0;
    int num1 = 0, num2 = 0, num3 = 0;
    for (auto &landmark : map_->landmarks()) {
        auto mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        // 未参与优化的无效路标点
        // Only those in the sliding window
        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        // 路标点在滑动窗口内的所有观测
        // All the observations for mappoint
        vector<double> errors;
        for (auto &observation : mappoint->observations()) {
            auto feat = observation.lock();
            if (!feat || feat->isOutlier()) {
                continue;
            }
            auto frame = feat->getFrame();
            if (!frame || !frame->isKeyFrame() || !map_->isKeyFrameInMap(frame)) {
                continue;
            }

            auto pp = feat->keyPoint();

            // 计算重投影误差
            // Calculate the reprojection error
            double error = camera_->reprojectionError(frame->pose(), mappoint->pos(), pp).norm();

            // 大于3倍阈值, 则禁用当前观测
            // Feature outlier
            if (!tracking_->isGoodToTrack(pp, frame->pose(), mappoint->pos(), 3.0)) {
                feat->setOutlier(true);
                mappoint->decreaseUsedTimes();

                // 如果当前观测帧是路标点的参考帧, 直接设置为outlier
                // Mappoint
                if (frame->id() == mappoint->referenceFrameId()) {
                    mappoint->setOutlier(true);
                    mappoints.push_back(mappoint);
                    num_outliers_mappoint++;
                    num1++;
                    break;
                }
                num_outliers_feature++;
            } else {
                errors.push_back(error);
            }
        }

        // 有效观测不足, 平均重投影误差较大, 则为粗差
        // Mappoint outlier
        if (errors.size() < 2) {
            mappoint->setOutlier(true);
            mappoints.push_back(mappoint);
            num_outliers_mappoint++;
            num2++;
        } else {
            double avg_error = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
            if (avg_error > reprojection_error_std_) {
                mappoint->setOutlier(true);
                mappoints.push_back(mappoint);
                num_outliers_mappoint++;
                num3++;
            }
        }
    }

    // 移除outliers
    // Remove the mappoint outliers
    for (auto &mappoint : mappoints) {
        map_->removeMappoint(mappoint);
    }

    LOGI << "Culled " << num_outliers_mappoint << " mappoint with " << num_outliers_feature << " bad observed features "
         << num1 << ", " << num2 << ", " << num3;
    outliers_[0] = num_outliers_mappoint;
    outliers_[1] = num_outliers_feature;

    return true;
}

bool GVINS::gvinsOptimization() {
    static int first_num_iterations  = optimize_num_iterations_ / 4;
    static int second_num_iterations = optimize_num_iterations_ - first_num_iterations;

    TimeCost timecost;

    ceres::Problem::Options problem_options;
    problem_options.enable_fast_removal = true;

    ceres::Problem problem(problem_options);
    ceres::Solver solver;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type         = ceres::DENSE_SCHUR;
    options.max_num_iterations         = first_num_iterations;
    options.num_threads                = 4;

    // 状态参数
    // State parameters
    addStateParameters(problem);

    // 重投影参数
    // Visual parameters
    addReprojectionParameters(problem);

    // 边缘化残差
    // The prior factor
    if (last_marginalization_info_ && last_marginalization_info_->isValid()) {
        auto factor = new MarginalizationFactor(last_marginalization_info_);
        problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks_);
    }

    // GNSS残差
    // The GNSS factors
    auto gnss_redisual_block = addGnssFactors(problem, true);

    // NC-IC extension: calibrated heading is an optional yaw-only external
    // observation and stays dormant for datasets without magnetometer data.
    addHeadingFactors(problem, true);

    // 预积分残差
    // The IMU preintegration factors
    addImuFactors(problem);

    // 视觉重投影残差
    // The visual reprojection factors
    auto residual_ids = addReprojectionFactors(problem, true);

    LOGI << "Add " << preintegrationlist_.size() << " preintegration, " << gnsslist_.size() << " GNSS, "
         << residual_ids.size() << " reprojection";

    // 第一次优化
    // The first optimization
    {
        timecost.restart();

        solver.Solve(options, &problem, &summary);
        LOGI << summary.BriefReport();

        iterations_[0] = summary.num_successful_steps;
        timecosts_[0]  = timecost.costInMillisecond();
    }

    // 粗差检测和剔除
    // Outlier detetion for GNSS and visual
    {
        // Remove factors in the final

        // Do GNSS outlier culling
        gnssOutlierCullingByChi2(problem, gnss_redisual_block);

        // Remove outlier reprojection factors
        const int visual_outliers = removeReprojectionFactorsByChi2(problem, residual_ids, 5.991);
        if (nc_extension_enabled_ && gvinsstate_ >= GVINS_TRACKING_INITIALIZING) {
            VisualQualityReport report;
            report.time = timelist_.back();
            report.residual_count = static_cast<int>(residual_ids.size());
            report.outlier_count = visual_outliers;
            report.outlier_ratio = residual_ids.empty()
                                       ? 1.0
                                       : static_cast<double>(visual_outliers) /
                                             static_cast<double>(residual_ids.size());
            report.valid = report.residual_count >= visual_min_residuals_ &&
                           report.outlier_ratio <= visual_max_outlier_ratio_;
            SensorHealthState previous_visual_health;
            ModalityDecision decision;
            {
                std::lock_guard<std::mutex> health_lock(health_mutex_);
                previous_visual_health = gnss_health_manager_.visionState();
                decision =
                    gnss_health_manager_.updateVision(report.time, report.valid, report.outlier_ratio);
            }
            visual_factor_std_scale_ = decision.covariance_scale;
            std::vector<std::string> visual_reasons;
            if (report.residual_count < visual_min_residuals_) {
                visual_reasons.emplace_back(
                    absl::StrFormat("visual residual count %d below %d",
                                    report.residual_count, visual_min_residuals_));
            }
            if (report.outlier_ratio > visual_max_outlier_ratio_) {
                visual_reasons.emplace_back(
                    absl::StrFormat("visual outlier ratio %.3lf exceeds %.3lf",
                                    report.outlier_ratio, visual_max_outlier_ratio_));
            }
            logSensorHealthTransition(
                "vision", previous_visual_health, decision.health.state,
                joinReasons(visual_reasons),
                absl::StrFormat("visual residuals %d and outlier ratio %.3lf passed recovery gate, samples=%d",
                                report.residual_count, report.outlier_ratio,
                                decision.health.recovery_samples),
                report.time);
            emitHealthStatus(report.time);
        }

        // Remove all GNSS factors
        for (auto &block : gnss_redisual_block) {
            problem.RemoveResidualBlock(block.first);
        }

        // Add GNSS Factors without loss function
        addGnssFactors(problem, false);
    }

    // 第二次优化
    // The second optimization
    {
        options.max_num_iterations = second_num_iterations;

        timecost.restart();

        solver.Solve(options, &problem, &summary);
        LOGI << summary.BriefReport();

        iterations_[1] = summary.num_successful_steps;
        timecosts_[1]  = timecost.costInMillisecond();

        if (!map_->isMaximumKeframes()) {
            // 进行必要的重积分
            // Reintegration during initialization
            doReintegration();
        }
    }

    // 更新参数, 必须的
    // Update the parameters from the optimizer
    updateParametersFromOptimizer();

    // 移除粗差路标点
    // Remove mappoint and feature outliers
    gvinsOutlierCulling();

    return true;
}

void GVINS::gnssOutlierCullingByChi2(ceres::Problem &problem,
                                     vector<std::pair<ceres::ResidualBlockId, GNSS *>> &redisual_block) {
    double cost, chi2;

    int outliers_counts = 0;
    for (auto &block : redisual_block) {
        auto id    = block.first;
        GNSS *gnss = block.second;

        problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);
        chi2 = cost * 2;

        // NC-IC extension: original IC tested a fixed 3-D GNSS residual.
        // Vertically rejected observations now contain only two informative
        // rows, so their chi-square gate and reweighting must be axis aware.
        const int dof = (gnss->horizontal_valid ? 2 : 0) + (gnss->vertical_valid ? 1 : 0);
        if (dof == 0) {
            continue;
        }
        const double chi2_threshold = (dof == 1) ? 3.841 : ((dof == 2) ? 5.991 : 7.815);
        if (chi2 > chi2_threshold) {

            // Reweigthed GNSS
            double scale = sqrt(chi2 / chi2_threshold);
            if (gnss->horizontal_valid) {
                gnss->std[0] *= scale;
                gnss->std[1] *= scale;
            }
            if (gnss->vertical_valid) {
                gnss->std[2] *= scale;
            }

            outliers_counts++;
        }
    }

    if (outliers_counts) {
        LOGI << "Detect " << outliers_counts << " GNSS outliers at " << Logging::doubleData(timelist_.back());
    }
}

int GVINS::removeReprojectionFactorsByChi2(ceres::Problem &problem, vector<ceres::ResidualBlockId> &residual_ids,
                                           double chi2) {
    double cost;
    int outlier_features = 0;

    // 进行卡方检验, 判定粗差因子, 待全部判定完成再进行移除, 否则会导致错误
    // Judge first and remove later
    vector<ceres::ResidualBlockId> outlier_residual_ids;
    for (auto &id : residual_ids) {
        problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);

        // cost带有1/2系数
        // To chi2
        if (cost * 2.0 > chi2) {
            outlier_features++;
            outlier_residual_ids.push_back(id);
        }
    }

    // 从优化问题中移除所有粗差因子
    // Remove the outliers from the optimizer
    for (auto &id : outlier_residual_ids) {
        problem.RemoveResidualBlock(id);
    }

    LOGI << "Remove " << outlier_features << " reprojection factors";

    return outlier_features;
}

void GVINS::updateParametersFromOptimizer() {
    if (map_->keyframes().empty()) {
        return;
    }

    // 先更新外参, 更新位姿需要外参
    // Update the extrinsic first
    {
        if (optimize_estimate_td_) {
            td_b_c_ = extrinsic_[7];
        }

        if (optimize_estimate_extrinsic_) {
            Pose ext;
            ext.t[0] = extrinsic_[0];
            ext.t[1] = extrinsic_[1];
            ext.t[2] = extrinsic_[2];

            Quaterniond qic = Quaterniond(extrinsic_[6], extrinsic_[3], extrinsic_[4], extrinsic_[5]);
            ext.R           = Rotation::quaternion2matrix(qic.normalized());

            // 外参估计检测, 误差较大则不更新, 1m or 5deg
            double dt = (ext.t - pose_b_c_.t).norm();
            double dr = Rotation::matrix2quaternion(ext.R * pose_b_c_.R.transpose()).vec().norm() * R2D;
            if ((dt > 1.0) || (dr > 5.0)) {
                LOGE << "Estimated extrinsic is too large, t: " << ext.t.transpose()
                     << ", R: " << Rotation::matrix2euler(ext.R).transpose() * R2D;
            } else {
                // Update the extrinsic
                Lock lock(extrinsic_mutex_);
                pose_b_c_ = ext;
            }

            vector<double> extrinsic;
            Vector3d euler = Rotation::matrix2euler(ext.R) * R2D;

            extrinsic.push_back(timelist_.back());
            extrinsic.push_back(ext.t[0]);
            extrinsic.push_back(ext.t[1]);
            extrinsic.push_back(ext.t[2]);
            extrinsic.push_back(euler[0]);
            extrinsic.push_back(euler[1]);
            extrinsic.push_back(euler[2]);
            extrinsic.push_back(td_b_c_);

            extfilesaver_->dump(extrinsic);
            extfilesaver_->flush();
        }
    }

    // 更新关键帧的位姿
    // Update the keyframe pose
    for (auto &keyframe : map_->keyframes()) {
        auto &frame = keyframe.second;
        auto index  = getStateDataIndex(frame->stamp());
        if (index < 0) {
            continue;
        }

        IntegrationState state = Preintegration::stateFromData(statedatalist_[index], preintegration_options_);
        frame->setPose(MISC::stateToCameraPose(state, pose_b_c_));
    }

    // 更新路标点的深度和位置
    // Update the mappoints
    for (const auto &landmark : map_->landmarks()) {
        const auto &mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        auto frame = mappoint->referenceFrame();
        if (!frame || !map_->isKeyFrameInMap(frame)) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        double invdepth = invdepthlist_[mappoint->id()];
        double depth    = 1.0 / invdepth;

        auto pc0      = camera_->pixel2cam(mappoint->referenceKeypoint());
        Vector3d pc00 = {pc0.x(), pc0.y(), 1.0};
        pc00 *= depth;

        mappoint->pos() = camera_->cam2world(pc00, mappoint->referenceFrame()->pose());
        mappoint->updateDepth(depth);
    }
}

bool GVINS::gvinsRemoveAllSecondNewFrame() {
    vector<ulong> keyframeids = map_->orderedKeyFrames();

    for (auto id : keyframeids) {
        auto frame = map_->keyframes().find(id)->second;
        // 移除次新帧, 以及倒数第二个空关键帧
        if ((frame->keyFrameState() == KEYFRAME_REMOVE_SECOND_NEW) ||
            (frame->features().empty() && (id != keyframeids.back()))) {
            unused_time_nodes_.push_back(frame->stamp());

            // 仅需要重置关键帧标志, 从地图中移除次新关键帧即可,
            // 无需调整状态参数和路标点
            // Just remove the frame
            frame->resetKeyFrame();
            map_->removeKeyFrame(frame, false);
        }
    }

    return true;
}

bool GVINS::gvinsMarginalization() {

    // 按时间先后排序的关键帧
    // Ordered keyframes
    vector<ulong> keyframeids = map_->orderedKeyFrames();
    auto latest_keyframe      = map_->latestKeyFrame();

    latest_keyframe->setKeyFrameState(KEYFRAME_NORMAL);

    // 对齐到保留的最后一个关键帧, 可能移除多个预积分对象
    // Align to the last keyframe time
    auto frame      = map_->keyframes().find(keyframeids[1])->second;
    size_t num_marg = getStateDataIndex(frame->stamp());

    double last_time = timelist_[num_marg];

    LOGI << "Marginalize " << num_marg << " states, last time " << Logging::doubleData(last_time);

    std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>();

    // 指定每个参数块独立的ID, 用于索引参数
    // For fixed order
    std::unordered_map<long, long> parameters_ids;
    parameters_ids.clear();
    long parameters_id = 0;

    {
        // 边缘化参数
        // Marginalization parameters
        for (auto &last_marginalization_parameter_block : last_marginalization_parameter_blocks_) {
            parameters_ids[reinterpret_cast<long>(last_marginalization_parameter_block)] = parameters_id++;
        }

        // 外参参数
        // Extrinsic parameters
        parameters_ids[reinterpret_cast<long>(extrinsic_)]     = parameters_id++;
        parameters_ids[reinterpret_cast<long>(extrinsic_ + 7)] = parameters_id++;

        // 位姿参数
        // Pose parameters
        for (const auto &statedata : statedatalist_) {
            parameters_ids[reinterpret_cast<long>(statedata.pose)] = parameters_id++;
            parameters_ids[reinterpret_cast<long>(statedata.mix)]  = parameters_id++;
        }

        // NC-IC extension: height-bias states are independent GNSS parameters
        // and must be assigned stable marginalization IDs just like poses.
        if (enable_height_bias_) {
            for (auto &gnss : gnsslist_) {
                if (gnss.use_height_bias) {
                    const long address = reinterpret_cast<long>(&gnss.height_bias);
                    if (parameters_ids.find(address) == parameters_ids.end()) {
                        parameters_ids[address] = parameters_id++;
                    }
                }
            }
        }

        // 逆深度参数
        // Inverse depth parameters
        frame         = map_->keyframes().at(keyframeids[0]);
        auto features = frame->features();
        for (auto const &feature : features) {
            auto mappoint = feature.second->getMapPoint();
            if (feature.second->isOutlier() || !mappoint || mappoint->isOutlier()) {
                continue;
            }

            if (mappoint->referenceFrame() != frame) {
                continue;
            }

            double *invdepth                                 = &invdepthlist_[mappoint->id()];
            parameters_ids[reinterpret_cast<long>(invdepth)] = parameters_id++;
        }

        // 更新参数块的特定ID, 必要的
        // Update the IS for parameters
        marginalization_info->updateParamtersIds(parameters_ids);
    }

    // 边缘化因子
    // The prior factor
    if (last_marginalization_info_ && last_marginalization_info_->isValid()) {

        std::vector<int> marginalized_index;
        for (size_t i = 0; i < num_marg; i++) {
            for (size_t k = 0; k < last_marginalization_parameter_blocks_.size(); k++) {
                if (last_marginalization_parameter_blocks_[k] == statedatalist_[i].pose ||
                    last_marginalization_parameter_blocks_[k] == statedatalist_[i].mix) {
                    marginalized_index.push_back((int) k);
                }
            }
        }
        if (enable_height_bias_) {
            for (auto &gnss : gnsslist_) {
                if (!gnss.use_height_bias || gnss.time > last_time) {
                    continue;
                }
                for (size_t k = 0; k < last_marginalization_parameter_blocks_.size(); k++) {
                    if (last_marginalization_parameter_blocks_[k] == &gnss.height_bias) {
                        // NC-IC extension: height bias is a state in the
                        // carried prior. Eliminate it before its owning GNSS
                        // deque element is popped, avoiding a stale block.
                        marginalized_index.push_back(static_cast<int>(k));
                    }
                }
            }
        }

        auto factor   = std::make_shared<MarginalizationFactor>(last_marginalization_info_);
        auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr, last_marginalization_parameter_blocks_,
                                                            marginalized_index);
        marginalization_info->addResidualBlockInfo(residual);
    }

    // GNSS因子
    // The GNSS factors
    for (auto &gnss : gnsslist_) {
        for (size_t k = 0; k < num_marg; k++) {
            if (MISC::isTheSameTimeNode(gnss.time, timelist_[k], MISC::MINIMUM_TIME_INTERVAL)) {
                std::shared_ptr<ceres::CostFunction> factor;
                std::vector<double *> parameters{statedatalist_[k].pose};
                std::vector<int> marginalized{0};
                if (enable_height_bias_ && gnss.use_height_bias && gnss.vertical_valid) {
                    factor = std::make_shared<HeightBiasGnssFactor>(gnss, antlever_);
                    parameters.push_back(&gnss.height_bias);
                    marginalized.push_back(1);
                } else if (gnss.use_online_offset && gnss.recovery_deviation.valid) {
                    factor = std::make_shared<RecoveryGnssFactor>(gnss, antlever_);
                } else {
                    factor = std::make_shared<GnssFactor>(gnss, antlever_);
                }
                auto residual = std::make_shared<ResidualBlockInfo>(
                    factor, nullptr, parameters, marginalized);
                marginalization_info->addResidualBlockInfo(residual);
                break;
            }
        }
    }

    if (enable_height_bias_) {
        GNSS *previous_height = nullptr;
        for (auto &gnss : gnsslist_) {
            if (!gnss.use_height_bias || !gnss.vertical_valid) {
                continue;
            }
            if (!previous_height && gnss.time <= last_time && !height_bias_prior_marginalized_) {
                auto prior = std::make_shared<ceres::AutoDiffCostFunction<HeightBiasPriorFactor, 1, 1>>(
                    new HeightBiasPriorFactor(0.0, height_bias_prior_std_));
                auto residual = std::make_shared<ResidualBlockInfo>(
                    prior, nullptr, std::vector<double *>{&gnss.height_bias}, std::vector<int>{0});
                marginalization_info->addResidualBlockInfo(residual);
                height_bias_prior_marginalized_ = true;
            }
            if (previous_height && previous_height->time <= last_time) {
                auto random_walk =
                    std::make_shared<ceres::AutoDiffCostFunction<HeightBiasRandomWalkFactor, 1, 1, 1>>(
                        new HeightBiasRandomWalkFactor(height_bias_random_walk_std_,
                                                       gnss.time - previous_height->time));
                std::vector<int> marginalized{0};
                if (gnss.time <= last_time) {
                    marginalized.push_back(1);
                }
                auto residual = std::make_shared<ResidualBlockInfo>(
                    random_walk, nullptr,
                    std::vector<double *>{&previous_height->height_bias, &gnss.height_bias},
                    marginalized);
                marginalization_info->addResidualBlockInfo(residual);
            }
            previous_height = &gnss;
        }
    }

    if (use_magnetic_heading_) {
        for (const auto &heading : headinglist_) {
            for (size_t k = 0; k < num_marg; k++) {
                if (MISC::isTheSameTimeNode(heading.time, timelist_[k], MISC::MINIMUM_TIME_INTERVAL)) {
                    auto factor =
                        std::make_shared<ceres::AutoDiffCostFunction<HeadingFactor, 1, 7>>(
                            new HeadingFactor(heading.yaw, heading.std));
                    auto residual = std::make_shared<ResidualBlockInfo>(
                        factor, nullptr, std::vector<double *>{statedatalist_[k].pose},
                        std::vector<int>{0});
                    marginalization_info->addResidualBlockInfo(residual);
                    break;
                }
            }
        }
    }

    // 预积分因子
    // The IMU preintegration factors
    for (size_t k = 0; k < num_marg; k++) {
        // 由于会移除多个预积分, 会导致出现保留和移除同时出现, 判断索引以区分
        // More than one may be removed
        vector<int> marg_index;
        if (k == (num_marg - 1)) {
            marg_index = {0, 1};
        } else {
            marg_index = {0, 1, 2, 3};
        }

        auto factor   = std::make_shared<PreintegrationFactor>(preintegrationlist_[k]);
        auto residual = std::make_shared<ResidualBlockInfo>(
            factor, nullptr,
            std::vector<double *>{statedatalist_[k].pose, statedatalist_[k].mix, statedatalist_[k + 1].pose,
                                  statedatalist_[k + 1].mix},
            marg_index);
        marginalization_info->addResidualBlockInfo(residual);
    }

    // 先验约束因子
    // The prior factor
    if (is_use_prior_) {
        auto pose_factor   = std::make_shared<ImuPosePriorFactor>(pose_prior_, pose_prior_std_);
        auto pose_residual = std::make_shared<ResidualBlockInfo>(
            pose_factor, nullptr, std::vector<double *>{statedatalist_[0].pose}, vector<int>{0});
        marginalization_info->addResidualBlockInfo(pose_residual);

        auto mix_factor   = std::make_shared<ImuMixPriorFactor>(preintegration_options_, mix_prior_, mix_prior_std_);
        auto mix_residual = std::make_shared<ResidualBlockInfo>(
            mix_factor, nullptr, std::vector<double *>{statedatalist_[0].mix}, vector<int>{0});
        marginalization_info->addResidualBlockInfo(mix_residual);

        is_use_prior_ = false;
    }

    // 重投影因子, 最老的关键帧
    // The visual reprojection factors

    frame         = map_->keyframes().at(keyframeids[0]);
    auto features = frame->features();

    auto loss_function = std::make_shared<ceres::HuberLoss>(1.0);
    for (auto const &feature : features) {
        auto mappoint = feature.second->getMapPoint();
        if (feature.second->isOutlier() || !mappoint || mappoint->isOutlier()) {
            continue;
        }

        auto ref_frame = mappoint->referenceFrame();
        if (ref_frame != frame) {
            continue;
        }

        auto ref_frame_pc      = camera_->pixel2cam(mappoint->referenceKeypoint());
        size_t ref_frame_index = getStateDataIndex(ref_frame->stamp());
        if (ref_frame_index < 0) {
            continue;
        }

        double *invdepth = &invdepthlist_[mappoint->id()];

        auto ref_feature = ref_frame->features().find(mappoint->id())->second;

        auto observations = mappoint->observations();
        for (auto &observation : observations) {
            auto obs_feature = observation.lock();
            if (!obs_feature || obs_feature->isOutlier()) {
                continue;
            }
            auto obs_frame = obs_feature->getFrame();
            if (!obs_frame || !obs_frame->isKeyFrame() || !map_->isKeyFrameInMap(obs_frame) ||
                (obs_frame == ref_frame)) {
                continue;
            }

            auto obs_frame_pc      = camera_->pixel2cam(obs_feature->keyPoint());
            size_t obs_frame_index = getStateDataIndex(obs_frame->stamp());

            if ((obs_frame_index < 0) || (ref_frame_index == obs_frame_index)) {
                LOGE << "Wrong matched mapoint keyframes " << Logging::doubleData(ref_frame->stamp()) << " with "
                     << Logging::doubleData(obs_frame->stamp());
                continue;
            }

            auto factor = std::make_shared<ReprojectionFactor>(
                ref_frame_pc, obs_frame_pc, ref_feature->velocityInPixel(), obs_feature->velocityInPixel(),
                ref_frame->timeDelay(), obs_frame->timeDelay(),
                optimize_reprojection_error_std_ * visual_factor_std_scale_);
            auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr,
                                                                vector<double *>{statedatalist_[ref_frame_index].pose,
                                                                                 statedatalist_[obs_frame_index].pose,
                                                                                 extrinsic_, invdepth, &extrinsic_[7]},
                                                                vector<int>{0, 3});
            marginalization_info->addResidualBlockInfo(residual);
        }
    }

    // 边缘化处理
    // Do marginalization
    marginalization_info->marginalization();

    // 保留的数据, 使用独立ID
    // Update the address
    std::unordered_map<long, double *> address;
    for (size_t k = num_marg; k < statedatalist_.size(); k++) {
        address[parameters_ids[reinterpret_cast<long>(statedatalist_[k].pose)]] = statedatalist_[k].pose;
        address[parameters_ids[reinterpret_cast<long>(statedatalist_[k].mix)]]  = statedatalist_[k].mix;
    }
    address[parameters_ids[reinterpret_cast<long>(extrinsic_)]]     = extrinsic_;
    address[parameters_ids[reinterpret_cast<long>(extrinsic_ + 7)]] = &extrinsic_[7];
    if (enable_height_bias_) {
        for (auto &gnss : gnsslist_) {
            if (gnss.use_height_bias && gnss.time > last_time) {
                const long bias_address = reinterpret_cast<long>(&gnss.height_bias);
                auto id = parameters_ids.find(bias_address);
                if (id != parameters_ids.end()) {
                    address[id->second] = &gnss.height_bias;
                }
            }
        }
    }

    last_marginalization_parameter_blocks_ = marginalization_info->getParamterBlocks(address);
    last_marginalization_info_             = std::move(marginalization_info);

    // 移除边缘化的数据
    // Remove the marginalized data

    // GNSS观测
    // The GNSS observations
    size_t num_gnss = gnsslist_.size();
    for (size_t k = 0; k < gnsslist_.size(); k++) {
        if (gnsslist_[k].time > last_time) {
            num_gnss = k;
            break;
        }
    }
    for (size_t k = 0; k < num_gnss; k++) {
        gnsslist_.pop_front();
    }
    if (enable_height_bias_) {
        bool has_retained_height_bias = false;
        for (const auto &gnss : gnsslist_) {
            if (gnss.use_height_bias && gnss.vertical_valid) {
                has_retained_height_bias = true;
                break;
            }
        }
        if (!has_retained_height_bias) {
            // NC-IC extension: a later independent valid-height segment needs
            // its own weak zero prior after the earlier bias chain is gone.
            height_bias_prior_marginalized_ = false;
        }
    }

    while (!headinglist_.empty() && headinglist_.front().time <= last_time) {
        headinglist_.pop_front();
    }

    // 预积分观测及时间状态
    // The IMU preintegration and time nodes
    for (size_t k = 0; k < num_marg; k++) {
        timelist_.pop_front();
        statedatalist_.pop_front();
        preintegrationlist_.pop_front();
    }

    // 保存移除的路标点, 用于可视化
    // The marginalized mappoints, for visualization
    frame    = map_->keyframes().at(keyframeids[0]);
    features = frame->features();
    for (const auto &feature : features) {
        auto mappoint = feature.second->getMapPoint();
        if (feature.second->isOutlier() || !mappoint || mappoint->isOutlier()) {
            continue;
        }
        auto &pw = mappoint->pos();

        if (is_use_visualization_) {
            drawer_->addNewFixedMappoint(pw);
        }

        // 保存路标点
        // Save these mappoints to file
        ptsfilesaver_->dump(vector<double>{pw.x(), pw.y(), pw.z()});
    }

    // 关键帧
    // The marginalized keyframe
    map_->removeKeyFrame(frame, true);

    return true;
}

void GVINS::doReintegration() {
    int cnt = 0;
    for (size_t k = 0; k < preintegrationlist_.size(); k++) {
        IntegrationState state = Preintegration::stateFromData(statedatalist_[k], preintegration_options_);
        Vector3d dbg           = preintegrationlist_[k]->deltaState().bg - state.bg;
        Vector3d dba           = preintegrationlist_[k]->deltaState().ba - state.ba;
        if ((dbg.norm() > 6 * integration_parameters_->gyr_bias_std) ||
            (dba.norm() > 6 * integration_parameters_->acc_bias_std)) {
            preintegrationlist_[k]->reintegration(state);
            cnt++;
        }
    }
    if (cnt) {
        LOGW << "Reintegration " << cnt << " preintegration";
    }
}

void GVINS::addReprojectionParameters(ceres::Problem &problem) {
    if (map_->landmarks().empty()) {
        return;
    }

    invdepthlist_.clear();
    for (const auto &landmark : map_->landmarks()) {
        const auto &mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            auto frame = mappoint->referenceFrame();
            if (!frame || !map_->isKeyFrameInMap(frame)) {
                continue;
            }

            double depth         = mappoint->depth();
            double inverse_depth = 1.0 / depth;

            // 确保深度数值有效
            // For valid mappoints
            if (std::isnan(inverse_depth)) {
                mappoint->setOutlier(true);
                LOGE << "Mappoint " << mappoint->id() << " is wrong with depth " << depth << " type "
                     << mappoint->mapPointType();
                continue;
            }

            invdepthlist_[mappoint->id()] = inverse_depth;
            problem.AddParameterBlock(&invdepthlist_[mappoint->id()], 1);

            mappoint->addOptimizedTimes();
        }
    }

    // 外参
    // Extrinsic parameters
    extrinsic_[0] = pose_b_c_.t[0];
    extrinsic_[1] = pose_b_c_.t[1];
    extrinsic_[2] = pose_b_c_.t[2];

    Quaterniond qic = Rotation::matrix2quaternion(pose_b_c_.R);
    qic.normalize();
    extrinsic_[3] = qic.x();
    extrinsic_[4] = qic.y();
    extrinsic_[5] = qic.z();
    extrinsic_[6] = qic.w();

    ceres::LocalParameterization *parameterization = new (PoseParameterization);
    problem.AddParameterBlock(extrinsic_, 7, parameterization);

    if (!optimize_estimate_extrinsic_ || gvinsstate_ != GVINS_TRACKING_NORMAL) {
        problem.SetParameterBlockConstant(extrinsic_);
    }

    // 时间延时
    // Time delay
    extrinsic_[7] = td_b_c_;
    problem.AddParameterBlock(&extrinsic_[7], 1);
    if (!optimize_estimate_td_ || gvinsstate_ != GVINS_TRACKING_NORMAL) {
        problem.SetParameterBlockConstant(&extrinsic_[7]);
    }
}

vector<ceres::ResidualBlockId> GVINS::addReprojectionFactors(ceres::Problem &problem, bool isusekernel) {

    vector<ceres::ResidualBlockId> residual_ids;

    if (map_->keyframes().empty()) {
        return residual_ids;
    }

    ceres::LossFunction *loss_function = nullptr;
    if (isusekernel) {
        loss_function = new ceres::HuberLoss(1.0);
    }

    residual_ids.clear();
    for (const auto &landmark : map_->landmarks()) {
        const auto &mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        auto ref_frame = mappoint->referenceFrame();
        if (!map_->isKeyFrameInMap(ref_frame)) {
            continue;
        }

        auto ref_frame_pc      = camera_->pixel2cam(mappoint->referenceKeypoint());
        size_t ref_frame_index = getStateDataIndex(ref_frame->stamp());
        if (ref_frame_index < 0) {
            continue;
        }

        double *invdepth = &invdepthlist_[mappoint->id()];
        if (*invdepth == 0) {
            *invdepth = 1.0 / MapPoint::DEFAULT_DEPTH;
        }

        auto ref_feature = ref_frame->features().find(mappoint->id())->second;

        auto observations = mappoint->observations();
        for (auto &observation : observations) {
            auto obs_feature = observation.lock();
            if (!obs_feature || obs_feature->isOutlier()) {
                continue;
            }
            auto obs_frame = obs_feature->getFrame();
            if (!obs_frame || !obs_frame->isKeyFrame() || !map_->isKeyFrameInMap(obs_frame) ||
                (obs_frame == ref_frame)) {
                continue;
            }

            auto obs_frame_pc      = camera_->pixel2cam(obs_feature->keyPoint());
            size_t obs_frame_index = getStateDataIndex(obs_frame->stamp());

            if ((obs_frame_index < 0) || (ref_frame_index == obs_frame_index)) {
                LOGE << "Wrong matched mapoint keyframes " << Logging::doubleData(ref_frame->stamp()) << " with "
                     << Logging::doubleData(obs_frame->stamp());
                continue;
            }

            auto factor = new ReprojectionFactor(ref_frame_pc, obs_frame_pc, ref_feature->velocityInPixel(),
                                                 obs_feature->velocityInPixel(), ref_frame->timeDelay(),
                                                 obs_frame->timeDelay(),
                                                 optimize_reprojection_error_std_ * visual_factor_std_scale_);
            auto residual_block_id =
                problem.AddResidualBlock(factor, loss_function, statedatalist_[ref_frame_index].pose,
                                         statedatalist_[obs_frame_index].pose, extrinsic_, invdepth, &extrinsic_[7]);
            residual_ids.push_back(residual_block_id);
        }
    }

    return residual_ids;
}

int GVINS::getStateDataIndex(double time) {

    size_t index = MISC::getStateDataIndex(timelist_, time, MISC::MINIMUM_TIME_INTERVAL);
    if (!MISC::isTheSameTimeNode(timelist_[index], time, MISC::MINIMUM_TIME_INTERVAL)) {
        LOGE << "Wrong matching time node " << Logging::doubleData(timelist_[index]) << " to "
             << Logging::doubleData(time);
        return -1;
    }
    return static_cast<int>(index);
}

void GVINS::addStateParameters(ceres::Problem &problem) {
    LOGI << "Total " << statedatalist_.size() << " pose states from "
         << Logging::doubleData(statedatalist_.begin()->time) << " to "
         << Logging::doubleData(statedatalist_.back().time);

    for (auto &statedata : statedatalist_) {
        // 位姿
        // Pose
        ceres::LocalParameterization *parameterization = new (PoseParameterization);
        problem.AddParameterBlock(statedata.pose, Preintegration::numPoseParameter(), parameterization);

        // IMU mix parameters
        problem.AddParameterBlock(statedata.mix, Preintegration::numMixParameter(preintegration_options_));
    }
}

void GVINS::addImuFactors(ceres::Problem &problem) {
    for (size_t k = 0; k < preintegrationlist_.size(); k++) {
        // 预积分因子
        // IMU preintegration factors
        auto factor = new PreintegrationFactor(preintegrationlist_[k]);
        problem.AddResidualBlock(factor, nullptr, statedatalist_[k].pose, statedatalist_[k].mix,
                                 statedatalist_[k + 1].pose, statedatalist_[k + 1].mix);
    }

    // 添加IMU误差约束, 限制过大的误差估计
    // IMU error factor
    auto factor = new ImuErrorFactor(preintegration_options_);
    problem.AddResidualBlock(factor, nullptr, statedatalist_[preintegrationlist_.size()].mix);

    // IMU初始先验因子, 仅限于初始化
    // IMU prior factor, only for initialization
    if (is_use_prior_) {
        auto pose_factor = new ImuPosePriorFactor(pose_prior_, pose_prior_std_);
        problem.AddResidualBlock(pose_factor, nullptr, statedatalist_[0].pose);

        auto mix_factor = new ImuMixPriorFactor(preintegration_options_, mix_prior_, mix_prior_std_);
        problem.AddResidualBlock(mix_factor, nullptr, statedatalist_[0].mix);
    }
}

vector<std::pair<ceres::ResidualBlockId, GNSS *>> GVINS::addGnssFactors(ceres::Problem &problem, bool isusekernel) {
    vector<std::pair<ceres::ResidualBlockId, GNSS *>> residual_block;

    ceres::LossFunction *loss_function = nullptr;
    if (isusekernel) {
        loss_function = new ceres::HuberLoss(1.0);
    }

    GNSS *previous_height = nullptr;
    for (auto &data : gnsslist_) {
        int index = getStateDataIndex(data.time);
        if (index >= 0) {
            ceres::ResidualBlockId id;
            if (enable_height_bias_ && data.use_height_bias && data.vertical_valid) {
                // NC-IC extension: slow vertical bias is an explicit scalar
                // parameter.  It does not alter raw GNSS used in the map graph
                // and is present only when the vertical channel is admitted.
                problem.AddParameterBlock(&data.height_bias, 1);
                auto factor = new HeightBiasGnssFactor(data, antlever_);
                id = problem.AddResidualBlock(factor, loss_function, statedatalist_[index].pose,
                                              &data.height_bias);
                if (isusekernel) {
                    // The original optimizer re-adds GNSS measurement factors
                    // for its second pass.  Add random-walk/prior constraints
                    // only once so the NC bias model is not double weighted.
                    if (previous_height) {
                        auto random_walk =
                            new ceres::AutoDiffCostFunction<HeightBiasRandomWalkFactor, 1, 1, 1>(
                                new HeightBiasRandomWalkFactor(height_bias_random_walk_std_,
                                                               data.time - previous_height->time));
                        problem.AddResidualBlock(random_walk, nullptr, &previous_height->height_bias,
                                                 &data.height_bias);
                    } else if (!height_bias_prior_marginalized_) {
                        auto prior = new ceres::AutoDiffCostFunction<HeightBiasPriorFactor, 1, 1>(
                            new HeightBiasPriorFactor(0.0, height_bias_prior_std_));
                        problem.AddResidualBlock(prior, nullptr, &data.height_bias);
                    }
                    previous_height = &data;
                }
            } else if (data.use_online_offset && data.recovery_deviation.valid) {
                auto factor = new RecoveryGnssFactor(data, antlever_);
                id = problem.AddResidualBlock(factor, loss_function, statedatalist_[index].pose);
            } else {
                auto factor = new GnssFactor(data, antlever_);
                id = problem.AddResidualBlock(factor, loss_function, statedatalist_[index].pose);
            }
            residual_block.push_back(std::make_pair(id, &data));
        }
    }

    return residual_block;
}

void GVINS::addHeadingFactors(ceres::Problem &problem, bool isusekernel) {
    if (!use_magnetic_heading_) {
        return;
    }
    ceres::LossFunction *loss_function = isusekernel ? new ceres::HuberLoss(1.0) : nullptr;
    for (const auto &heading : headinglist_) {
        int index = getStateDataIndex(heading.time);
        if (index >= 0 && heading.valid && heading.std > 0) {
            // NC-IC extension: only calibrated yaw enters this factor.  Raw
            // magnetometer processing/calibration remains outside IC-GVINS.
            auto factor = new ceres::AutoDiffCostFunction<HeadingFactor, 1, 7>(
                new HeadingFactor(heading.yaw, heading.std));
            problem.AddResidualBlock(factor, loss_function, statedatalist_[index].pose);
        }
    }
}

void GVINS::constructPrior(bool is_zero_velocity) {
    double pos_prior_std  = 0.1;                                       // 0.1 m
    double att_prior_std  = 0.5 * D2R;                                 // 0.5 deg
    double vel_prior_std  = 0.1;                                       // 0.1 m/s
    double bg_prior_std   = integration_parameters_->gyr_bias_std * 3; // Bias std * 3
    double ba_prior_std   = ACCELEROMETER_BIAS_PRIOR_STD;              // 20000 mGal
    double sodo_prior_std = 0.005;                                     // 5000 PPM

    if (!is_zero_velocity) {
        bg_prior_std = GYROSCOPE_BIAS_PRIOR_STD; // 7200 deg/hr
    }

    memcpy(pose_prior_, statedatalist_[0].pose, sizeof(double) * 7);
    memcpy(mix_prior_, statedatalist_[0].mix, sizeof(double) * 18);
    for (size_t k = 0; k < 3; k++) {
        pose_prior_std_[k + 0] = pos_prior_std;
        pose_prior_std_[k + 3] = att_prior_std;

        mix_prior_std_[k + 0] = vel_prior_std;
        mix_prior_std_[k + 3] = bg_prior_std;
        mix_prior_std_[k + 6] = ba_prior_std;
    }
    pose_prior_std_[5] = att_prior_std * 3; // heading
    mix_prior_std_[9]  = sodo_prior_std;
    is_use_prior_      = true;
}
