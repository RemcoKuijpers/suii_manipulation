#include "rclcpp/rclcpp.hpp"
#include "config.h"
#include "robotControl.h"
#include "objectHandler.h"
#include <iostream>

// Define states
typedef enum {
    wait_for_service_call = 1,
    move,
    pick,
    place,
    done
} state_t;


int main(int argc, char **argv)
{
    // Initialize ROS and create node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("main");
    rclcpp::Rate r(EXECUTION_RATE);

    // Initialize UR3 connection    
    RobotControl ur3("192.168.178.71");
    std::cout << "Is robot connected ? " << ur3.isRobotConnected() << std::endl;

    // Initialize object handler
    ObjectHandler object_handler;

    // Initialize state machine
    state_t state = move;

    while(rclcpp::ok()){

        switch(state) {

            case wait_for_service_call:
                break;

            case move:
                ur3.closeGripper();
                if (ur3.moveL({-0.143, -0.435, 0.20, -0.001, 3.12, 0.04}, 0.5, 0.2)){
                    geometry_msgs::msg::Transform transform;
                    transform = object_handler.getTransform("ur3/tool0", "container_5");
                    std::cout << transform.translation.y << std::endl;
                    state = pick;
                }
                break;

            case pick:
                ur3.openGripper();
                if (ur3.moveJ(drive_pose, 0.5, 1)){
                    geometry_msgs::msg::Transform transform;
                    transform = object_handler.getTransform("ur3/tool0", "container_3");
                    std::cout << transform.translation.y << std::endl;
                    state = move;
                }
                break;

            case place:
                break;

            case done:
                break;
        }
        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}