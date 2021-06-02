#include "rclcpp/rclcpp.hpp"
#include "config.h"

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
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("Manipulation", node_options);
    rclcpp::Rate r(EXECUTION_RATE);

    // Initialize state machine
    state_t state = move;

    while(rclcpp::ok()){

        switch(state) {

            case wait_for_service_call:
                break;

            case move:
                break;

            case pick:
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