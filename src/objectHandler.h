#ifndef objectHandler_H
#define objectHandler_H


#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <iostream>

class ObjectHandler: public rclcpp::Node
{

public:
    ObjectHandler(): Node("object_handler"){
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    geometry_msgs::msg::Transform getTransform(const std::string targer_frame, const std::string source_frame);

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
};

#endif //objectHandler_H
