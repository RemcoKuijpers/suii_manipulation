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

std::string ObjectHandler::getEmptySpotOnRobot(){
    std::vector<std::string> frame_names;
    geometry_msgs::msg::TransformStamped transformStamped;
    bool container_1_occupied = false;
    bool container_2_occupied = false;
    bool container_3_occupied = false;
    std::string container_1 = "container_1";
    std::string container_2 = "container_2";
    std::string container_3 = "container_3";
    tf2::Transform t1, t2, t3;

    frame_names = this->tf_buffer_->getAllFrameNames();
    for(int i = 0 ; i < (int)frame_names.size() ; i++){
        t1 = this->getTransform(frame_names.at(i), container_1);
        t2 = this->getTransform(frame_names.at(i), container_2);
        t3 = this->getTransform(frame_names.at(i), container_3);
        if((abs(t1.getOrigin().getX()) < 0.001) && frame_names.at(i) != container_1) container_1_occupied = true;
        if((abs(t2.getOrigin().getX()) < 0.001) && frame_names.at(i) != container_2) container_2_occupied = true;
        if((abs(t3.getOrigin().getX()) < 0.001) && frame_names.at(i) != container_3) container_3_occupied = true;
    }
    if(!container_1_occupied) return "container_1";
    if(!container_2_occupied) return "container_2";
    if(!container_3_occupied) return "container_3";
    else return "None";
}