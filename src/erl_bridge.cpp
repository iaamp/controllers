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
    // std::cout << haptic_input_msg_.wrench.force.x() << std::endl;

    // lets check that we actually received a new message
    if (! ((haptic_input_msg_.time_sec == input_time_sec_) && (haptic_input_msg_.time_nsec == input_time_nsec_))) {
        // std::cout << "new message DroneFollower::Receive" << std::endl;
        last_input_ = std::chrono::high_resolution_clock::now();

        geometry_msgs::TwistStamped input_msg;
        geometry_msgs::Twist camera_msg;

        input_time_sec_ = haptic_input_msg_.time_sec;
        input_time_nsec_ = haptic_input_msg_.time_nsec;
        input_msg.twist.linear.x = haptic_input_msg_.wrench.force.x();
        input_msg.twist.linear.y = haptic_input_msg_.wrench.force.y();
        input_msg.twist.linear.z = haptic_input_msg_.wrench.force.z();
        input_msg.twist.angular.z = haptic_input_msg_.wrench.torque.z();
        input_msg.header.stamp.sec = input_time_sec_;
        input_msg.header.stamp.nsec = input_time_nsec_;

        camera_msg.angular.x = haptic_input_msg_.wrench.torque.x();
        camera_msg.angular.y = haptic_input_msg_.wrench.torque.y();
        // if (camera_msg.angular.x != 0 || camera_msg.angular.y != 0)
        if (input_msg.twist != input_msg_.twist)
            input_pub_.publish(input_msg);
        if (camera_msg != camera_msg_)
            camera_pub_.publish(camera_msg);

        input_msg_ = input_msg;
        camera_msg_ = camera_msg;
    } else {
        // we didn't receive a new message yet --> honour the timeout
        auto now = std::chrono::high_resolution_clock::now();
        if (now <= last_input_ + input_timeout_ms_) {
            input_pub_.publish(input_msg_);
            // if (camera_msg_.angular.x != 0 || camera_msg_.angular.y != 0)
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

void DroneFollower::start_recording()
{
    geometry_msgs::Twist camera_msg;
    camera_msg.linear.z = 2;
    camera_pub_.publish(camera_msg);
}

void DroneFollower::stop_recording()
{
    geometry_msgs::Twist camera_msg;
    camera_msg.linear.z = 3;
    camera_pub_.publish(camera_msg);
}

void DroneFollower::publish_camera_msg(const geometry_msgs::Twist& camera_msg)
{
    camera_pub_.publish(camera_msg);
}

ErlBridge::ErlBridge(ros::NodeHandle n, std::shared_ptr<DroneFollower> follower)
{
    node_handle_ = n;
    follower_ = follower;
    double value;
    std::string str;

    std::string n_namespace = n.getNamespace();
    std::cout << "****** n_namespace=" << n_namespace << std::endl;
    n.getParam(n_namespace+"/messenger_name",str);
    messenger_name_ = str;
    n.getParam(n_namespace+"/messenger_port",value);
    messenger_port_ = value;
    n.getParam(n_namespace+"/command_topic", str);
    command_topic_ = str;

    messenger_ = std::make_shared<RobotP2P::Messenger>(messenger_name_, messenger_port_);
    messenger_->set_spdlog_level(0);
    messenger_->bind_method("arm", std::bind(&ErlBridge::arm, this, std::placeholders::_1));
    messenger_->bind_method("stop_motors", std::bind(&ErlBridge::stop_motors, this, std::placeholders::_1));
    messenger_->bind_method("take_off", std::bind(&ErlBridge::take_off, this, std::placeholders::_1));
    messenger_->bind_method("land", std::bind(&ErlBridge::land, this, std::placeholders::_1));
    messenger_->bind_method("hover_at_position", std::bind(&ErlBridge::hover_at_position, this, std::placeholders::_1));
    messenger_->bind_method("start_recording", std::bind(&ErlBridge::start_recording, this, std::placeholders::_1));
    messenger_->bind_method("stop_recording", std::bind(&ErlBridge::stop_recording, this, std::placeholders::_1));
    messenger_->start_server();

    command_pub_ = n.advertise<std_msgs::Int8>(command_topic_,0);
}

bool ErlBridge::start_bridge()
{

}

bool ErlBridge::stop_bridge()
{

}

json ErlBridge::arm(json args)
{
    command_.data = 1;
    command_pub_.publish(command_);
    json result = {{"success", true}};
    return result;
}

json ErlBridge::take_off(json args)
{
    command_.data = 2;
    command_pub_.publish(command_);
    json result = {{"success", true}};
    return result;
}

json ErlBridge::hover_at_position(json args)
{
    command_.data = 3;
    command_pub_.publish(command_);
    json result = {{"success", true}};
    return result;
}

json ErlBridge::land(json args)
{
    command_.data = 4;
    command_pub_.publish(command_);
    json result = {{"success", true}};
    return result;
}

json ErlBridge::stop_motors(json args)
{
    command_.data = 5;
    command_pub_.publish(command_);
    json result = {{"success", true}};
    return result;
}

json ErlBridge::start_recording(json args)
{
    follower_->start_recording();
    json result = {{"success", true}};
    return result;
}

json ErlBridge::stop_recording(json args)
{
    follower_->stop_recording();
    json result = {{"success", true}};
    return result;
}

