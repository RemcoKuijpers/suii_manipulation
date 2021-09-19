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

    // Initialize state machine
    state_t state = wait_for_service_call;

    double object_translation1[3] = {0.5, 0.3, 0.15};
    double object_rotation1[3] = {-0.6, -0.4, M_PI_4};

    double object_translation2[3] = {0.5, -0.4, 0.05};
    double object_rotation2[3] = {0, 0, -0.5};

    std::vector<std::string> items_to_pick = {"OBJECT1", "OBJECT2"};

    while(rclcpp::ok()){

        switch(state) {

            case wait_for_service_call:
                ur3.object_handler.setFrame("base_link", "OBJECT1", object_translation1, object_rotation1);
                ur3.object_handler.setFrame("base_link", "OBJECT2", object_translation2, object_rotation2);
                state = move;
                break;

            case move:
                if(ur3.moveJ(drive_pose, 0.5, 1)){
                    state = pick;
                }
                break;

            case pick:
                for(int i = 0; i<(int)items_to_pick.size(); i++){
                    ur3.moveObject(items_to_pick[i], containers[i]);
                }
                state = wait_for_service_call;
                break;

            case place:
                break;

            case done:
                break;
        }
        r.sleep();
    }
    ur3.disconnectRobot();
    rclcpp::shutdown();
    return 0;
}