#include "objectHandler.h"

tf2::Transform ObjectHandler::getTransform(const std::string target_frame, const std::string source_frame){

    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = this->tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException & ex) {
          std::cout << "Could not transform " << target_frame.c_str() << " to " << source_frame.c_str() <<" : "<< ex.what() << std::endl;
    }
    tf2::Transform t;
    t.setOrigin(tf2::Vector3(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z));
    t.setRotation(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
    return t;
}

void ObjectHandler::setFrame(const std::string frame_id, const std::string child_frame_id, const double translation[3], const double rotation[3]){
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = frame_id;
    t.child_frame_id = child_frame_id;
    t.transform.translation.x = translation[0];
    t.transform.translation.y = translation[1];
    t.transform.translation.z = translation[2];
    tf2::Quaternion q;
    q.setRPY(rotation[0], rotation[1], rotation[2]);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_publisher_->sendTransform(t);
}