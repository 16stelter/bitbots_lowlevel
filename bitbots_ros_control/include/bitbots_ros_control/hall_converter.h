#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <bitbots_msgs/msg/float_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


/**
 * This class converts the raw readings of the Hall sensor into an rotational angle. 
 * An offset and a gradient are used to match the sensor readings to the readings of the actuator. 
 **/
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
    float gradient_;
};
