#include "rclcpp/rclcpp.hpp"
#include "robotControl.h"
#include "objectHandler.h"
#include <math.h>
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

    ObjectHandler object_handler;

    // Initialize state machine
    state_t state = wait_for_service_call;

    double object_translation1[3] = {0.5, 0.3, 0.15};
    double object_rotation1[3] = {-0.6, -0.4, M_PI_4};

    double object_translation2[3] = {0.5, -0.4, 0.05};
    double object_rotation2[3] = {0, 0, -0.5};

    while(rclcpp::ok()){

        switch(state) {

            case wait_for_service_call:
                object_handler.setFrame("base_link", "OBJECT1", object_translation1, object_rotation1);
                object_handler.setFrame("base_link", "OBJECT2", object_translation2, object_rotation2);
                state = move;
                break;

            case move:
                ur3.openGripper();
                if(ur3.moveJ(drive_pose, 0.5, 1)){
                    state = pick;
                }
                break;

            case pick:
                ur3.openGripper();
                if (ur3.pickObject("OBJECT1")){
                    state = place;
                }
                break;

            case place:
                ur3.openGripper();
                if (ur3.pickObject("container_1")){
                    state = done;
                }
                break;

            case done:
                ur3.openGripper();
                if (ur3.pickObject("OBJECT2")){
                    state = move;
                }
                break;
        }
        r.sleep();
    }
    ur3.disconnectRobot();
    rclcpp::shutdown();
    return 0;
}