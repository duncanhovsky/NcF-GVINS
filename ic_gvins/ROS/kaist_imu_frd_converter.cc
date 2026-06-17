#include "kaist_imu_frd_converter.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <algorithm>
#include <array>
#include <string>

namespace {

std::array<double, 9> toArray(const boost::array<double, 9> &input) {
    std::array<double, 9> output{};
    std::copy(input.begin(), input.end(), output.begin());
    return output;
}

void copyArray(const std::array<double, 9> &input, boost::array<double, 9> &output) {
    std::copy(input.begin(), input.end(), output.begin());
}

class KaistImuFrdConverter {
  public:
    explicit KaistImuFrdConverter(ros::NodeHandle private_node) : private_node_(private_node) {
        private_node_.param<std::string>("input_topic", input_topic_, "/imu/data_raw");
        private_node_.param<std::string>("output_topic", output_topic_, "/imu/data_frd");
        private_node_.param<std::string>("output_frame_id", output_frame_id_, "imu_frd");
        private_node_.param("invalidate_orientation", invalidate_orientation_, true);

        publisher_ = node_.advertise<sensor_msgs::Imu>(output_topic_, 200);
        subscriber_ = node_.subscribe(input_topic_, 200, &KaistImuFrdConverter::callback, this);

        ROS_INFO_STREAM("KAIST IMU FLU->FRD converter: " << input_topic_ << " -> " << output_topic_);
    }

  private:
    void callback(const sensor_msgs::ImuConstPtr &message) {
        sensor_msgs::Imu converted = *message;
        converted.header.frame_id  = output_frame_id_;

        const auto angular = kaist_imu_frd::fluToFrd({
            message->angular_velocity.x,
            message->angular_velocity.y,
            message->angular_velocity.z,
        });
        converted.angular_velocity.x = angular.x;
        converted.angular_velocity.y = angular.y;
        converted.angular_velocity.z = angular.z;

        const auto acceleration = kaist_imu_frd::fluToFrd({
            message->linear_acceleration.x,
            message->linear_acceleration.y,
            message->linear_acceleration.z,
        });
        converted.linear_acceleration.x = acceleration.x;
        converted.linear_acceleration.y = acceleration.y;
        converted.linear_acceleration.z = acceleration.z;

        copyArray(kaist_imu_frd::fluToFrdCovariance(toArray(message->angular_velocity_covariance)),
                  converted.angular_velocity_covariance);
        copyArray(kaist_imu_frd::fluToFrdCovariance(toArray(message->linear_acceleration_covariance)),
                  converted.linear_acceleration_covariance);

        if (invalidate_orientation_) {
            converted.orientation.x = 0.0;
            converted.orientation.y = 0.0;
            converted.orientation.z = 0.0;
            converted.orientation.w = 1.0;
            converted.orientation_covariance.assign(0.0);
            converted.orientation_covariance[0] = -1.0;
        }

        publisher_.publish(converted);
    }

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    std::string input_topic_;
    std::string output_topic_;
    std::string output_frame_id_;
    bool invalidate_orientation_{true};
};

} // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "kaist_imu_frd_converter");
    ros::NodeHandle private_node("~");
    KaistImuFrdConverter converter(private_node);
    ros::spin();
    return 0;
}
