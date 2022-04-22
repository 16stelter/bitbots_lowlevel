#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_HALL_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_HALL_HARDWARE_INTERFACE_H_

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

            bool init();

            void read(const rclcpp::Time &t, const rclcpp::Duration &dt);

            void write(const rclcpp::Time &t, const rclcpp::Duration &dt);
        private:
            rclcpp::Node::SharedPtr nh_;
            std::shared_ptr<DynamixelDriver> driver_;
            int id_;
            std::string topic_name_;
            std::string name_;
            uint8_t *data_;

            std::vector<float> current_angle_;
            bitbots_msgs::msg::FloatStamped msg_;
            rclcpp::Publisher<bitbots_msgs::msg::FloatStamped>::SharedPtr angle_pub_; //TODO: define angle message
    };
}

#endif
