#include <bitbots_ros_control/hall_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {

    HallHardwareInterface::HallHardwareInterface(rclcpp::Node::SharedPtr nh,
                                                       std::shared_ptr <DynamixelDriver> &driver,
                                                       int id,
                                                       std::string topic_name,
                                                       std::string name) {
        nh_ = nh;
        driver_ = driver;
        id_ = id;
        topic_name_ = topic_name;
        name_ = name;
    }

    bool HallHardwareInterface::init() {


        return true;
    }
}