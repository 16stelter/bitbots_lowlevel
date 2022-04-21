#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BITFOOT_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BITFOOT_HARDWARE_INTERFACE_H_

#include <rclcpp/rclcpp.hpp>
#include <string>

#include <bitbots_ros_control/hardware_interface.h>
#include <dynamixel_driver.h>

namespace bitbots_ros_control {

    class HallHardwareInterface : public bitbots_ros_control::HardwareInterface {
        public:
            explicit HallHardwareInterface(rclcpp::Node::SharedPtr nh,
                                           std::shared_ptr <DynamixelDriver> &driver,
                                           int id,
                                           std::string topic_name,
                                           std::string name);
        private:
    }
}

#endif
