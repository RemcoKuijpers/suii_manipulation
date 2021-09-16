#include "objectHandler.h"

geometry_msgs::msg::Transform ObjectHandler::getTransform(const std::string target_frame, const std::string source_frame){

    geometry_msgs::msg::TransformStamped transformStamped;
    geometry_msgs::msg::Transform transform;

    try {
        transformStamped = this->tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException & ex) {
          std::cout << "Could not transform " << target_frame.c_str() << " to " << source_frame.c_str() <<" : "<< ex.what() << std::endl;
    }
    return transformStamped.transform;
}