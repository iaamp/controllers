#include <controllers/erl_bridge.hpp>

DroneFollower::DroneFollower(ros::NodeHandle n)
{
    node_handle_ = n;
    std::vector<double> values;
    double value;
    std::string str;

    std::string n_namespace = n.getNamespace();
    std::cout << "****** n_namespace=" << n_namespace << std::endl;
    n.getParam(n_namespace+"/this_port",value);
    this_port_ = value;

    n.getParam(n_namespace+"/remote_id",remote_id_);
    n.getParam(n_namespace+"/remote_port",value);
    remote_port_ = value;

    n.getParam(n_namespace+"/input_topic", str);
    input_topic_ = str;
    n.getParam(n_namespace+"/camera_topic", str);
    camera_topic_ = str;
    n.getParam(n_namespace+"/feedback_topic", str);
    feedback_topic_ = n_namespace + str;

    input_pub_ = n.advertise<geometry_msgs::TwistStamped>(input_topic_,0);
    camera_pub_ = n.advertise<geometry_msgs::Twist>(camera_topic_,0);
    // init our timeout clock
    n.getParam(n_namespace+"/input_timeout_ms",value);
    input_timeout_ms_ = std::chrono::milliseconds { int(value) };
    last_input_ = std::chrono::high_resolution_clock::now();

    // feedback_sub_ = n.subscribe(feedback_topic_, 1, &DroneFollower::WrenchCallback, this); // --> we don't have feedback yet
}

void DroneFollower::Receive()
{
    haptic_input_msg_ = haptic_udp_client_->LatestHapticInput();
    std::cout << haptic_input_msg_.wrench.force.x() << std::endl;

    // lets check that we got a new message
    if ((haptic_input_msg_.time_sec != input_time_sec_) &&
        std::cout << "new message DroneFollower::Receive" << std::endl;
        (haptic_input_msg_.time_nsec != input_time_nsec_)) {
        last_input_ = std::chrono::high_resolution_clock::now();

        geometry_msgs::TwistStamped input_msg;
        geometry_msgs::Twist camera_msg;

        input_time_sec_ = haptic_input_msg_.time_sec;
        input_time_nsec_ = haptic_input_msg_.time_nsec;
        input_msg.twist.linear.x = haptic_input_msg_.wrench.force.x();
        input_msg.twist.linear.y = haptic_input_msg_.wrench.force.y();
        input_msg.twist.linear.z = haptic_input_msg_.wrench.force.z();
        input_msg.twist.angular.x = haptic_input_msg_.wrench.torque.x();
        // tf::vectorEigenToMsg(haptic_input_msg_.wrench.force, input_msg_.twist.linear);
        // tf::vectorEigenToMsg(haptic_input_msg_.wrench.torque, input_msg_.twist.angular);
        input_msg.header.stamp.sec = input_time_sec_;
        input_msg.header.stamp.nsec = input_time_nsec_;
        input_pub_.publish(input_msg);

        camera_msg.angular.x = haptic_input_msg_.wrench.torque.y();
        camera_msg.angular.y = haptic_input_msg_.wrench.torque.z();
        camera_pub_.publish(camera_msg);

        input_msg_ = input_msg;
        camera_msg_ = camera_msg;
    } else {
        // we didn't receive a new message yet --> honour the timeout
        auto now = std::chrono::high_resolution_clock::now();
        if (now <= last_input_ + input_timeout_ms_) {
            input_pub_.publish(input_msg_);
            camera_pub_.publish(camera_msg_);
        } else {
            // std::cout << "timeout DroneFollower::Receive" << std::endl;
            tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), input_msg_.twist.linear);
            tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), input_msg_.twist.angular);
            tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), camera_msg_.linear);
            tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), camera_msg_.angular);
        }
    }
}


void DroneFollower::publish_camera_msg(const geometry_msgs::Twist& camera_msg)
{
    camera_pub_.publish(camera_msg);
}
