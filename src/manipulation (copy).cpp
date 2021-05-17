#include "rclcpp/rclcpp.hpp"
#include "config.h"
#include "iostream"


class ManipulationNode : public rclcpp::Node
{
public:
    ManipulationNode() : Node("manipulation") {}
private:
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulationNode>();

    rclcpp::Rate loop_rate(EXECUTION_RATE);

    while(rclcpp::ok()){
        std::cout << "Execution rate = " << EXECUTION_RATE << std::endl;


        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}