#ifndef SRC_SEA_CORRECTION_HELPER_H
#define SRC_SEA_CORRECTION_HELPER_H

#include <rclcpp/rclcpp.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <bitbots_msgs/msg/float_stamped.hpp>


class SeaCorrectionHelper {
  public:
    explicit SeaCorrectionHelper(rclcpp::Node::SharedPtr nh);

    std::shared_ptr<rclcpp::Node> lknee_node_;
    std::shared_ptr<rclcpp::Node> rknee_node_;
  private:
    rclcpp::Node::SharedPtr nh_;

    void commandCb(const bitbots_msgs::msg::JointCommand msg);
    void stateCb(const sensor_msgs::msg::JointState msg);
    void hallLCb(const bitbots_msgs::msg::FloatStamped &msg);
    void hallRCb(const bitbots_msgs::msg::FloatStamped &msg);

    std::shared_ptr<control_toolbox::PidROS> lknee_pid_;
    std::shared_ptr<control_toolbox::PidROS> rknee_pid_;
    double hall_l_;
    double hall_r_;

    rclcpp::Subscription<bitbots_msgs::msg::FloatStamped>::SharedPtr sub_hall_l_;
    rclcpp::Subscription<bitbots_msgs::msg::FloatStamped>::SharedPtr sub_hall_r_;
    rclcpp::Subscription<bitbots_msgs::msg::JointCommand>::SharedPtr sub_command_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_state_;
    rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr pub_;

    bitbots_msgs::msg::JointCommand latched_command_;
};


#endif //SRC_SEA_CORRECTION_HELPER_H
