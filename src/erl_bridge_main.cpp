
#include <iostream>
#include <thread>

#include "ros/ros.h"
#include <haptic.hpp>
#include <controllers/erl_bridge.hpp>

static volatile bool interrupted = false;

void SignalHandler(int sig)
{
    std::cout << "Interrupted" << std::endl;
    interrupted = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "erl_bridge");
    ros::NodeHandle n(ros::this_node::getName());
    std::string n_namespace = n.getNamespace();

    double rate;
    n.getParam(n_namespace+"/rate", rate);
    ros::Rate loop_rate(rate);

    asio::io_context io_context;
    std::shared_ptr<haptic::HapticUdpFollower> drone_follower;
    drone_follower = std::make_shared<DroneFollower>(n);
    if (!drone_follower->Connect(io_context)) {
        std::cerr << "Follower could not connect with remote" << std::endl;
    }
    // erl_bridge = std::make_shared<ErlBridge>(n);
    // erl_bridge->start_bridge();

    // Signal handlers
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    while(ros::ok()) {
        if (interrupted) {
            std::cerr << "Stopping Follower" << std::endl;
            drone_follower->Stop();
            // erl_bridge->stop_bridge();
            io_context.stop();
            break;
        }

        // action
        try {
            // drone_follower->Send(); // no feedback yet
            drone_follower->Receive();
        } catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << "\n";
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
