#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <bitbots_msgs/msg/float_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class HallConverter {
  public:
    HallConverter(rclcpp::Node::SharedPtr nh, char side);
  private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::executors::StaticSingleThreadedExecutor sub_executor_;
    rclcpp::CallbackGroup::SharedPtr sub_cbg_;
    std::thread* sub_executor_thread_;

    rclcpp::Publisher<bitbots_msgs::msg::FloatStamped>::SharedPtr pub_;
    rclcpp::Subscription<bitbots_msgs::msg::FloatStamped>::SharedPtr sub_;

    void hallCb(bitbots_msgs::msg::FloatStamped raw);

    char side_;
    float offset_;
};
