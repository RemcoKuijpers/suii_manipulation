#ifndef objectHandler_H
#define objectHandler_H


#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <iostream>

class ObjectHandler: public rclcpp::Node
{

public:
    ObjectHandler(): Node("object_handler"){
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

    void setFrame(const std::string frame_id, const std::string child_frame_id, const double translation[3], const double rotation[3]);
    tf2::Transform getTransform(const std::string target_frame, const std::string source_frame);

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};

#endif //objectHandler_H
