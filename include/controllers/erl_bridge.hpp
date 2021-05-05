#pragma once
#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <nlohmann/json.hpp>
#include <robotp2p/robotp2p.hpp>
#include <haptic.hpp>

using json = nlohmann::json;

class DroneFollower : public haptic::HapticUdpFollower {
public:
    DroneFollower(ros::NodeHandle n);
    ~DroneFollower() {
        Stop();
    }
    void Receive();
    void publish_camera_msg(const geometry_msgs::Twist& camera_msg);

private:
    ros::NodeHandle node_handle_;
    ros::Publisher input_pub_;
    ros::Publisher camera_pub_;
    std::string input_topic_;
    std::string camera_topic_;
    std::string feedback_topic_;
    geometry_msgs::TwistStamped input_msg_;
    geometry_msgs::Twist camera_msg_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_input_;
    std::chrono::milliseconds input_timeout_ms_;
};

// class ErlBridge {
// public:
//     ErlBridge(ros::NodeHandle n);

//     bool start_bridge();
//     bool stop_bridge();

//     json arm(json args);
//     json stop_motors(json args);
//     json take_off(json args);
//     json land(json args);
//     json hover_at_position(json args);
//     json start_recording(json args);
//     json stop_recording(json args);

// private:
//     RobotP2P::Messenger messenger_;
//     ros::NodeHandle node_handle_;
//     ros::Publisher command_publisher_;
//     std_msgs::Int8 command_;
// };
