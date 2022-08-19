#include <bitbots_ros_control/hall_converter.h>
using std::placeholders::_1;
using std::placeholders::_2;


HallConverter::HallConverter(rclcpp::Node::SharedPtr nh, char side) {
    nh_ = nh;
    std::string topic;
    side_ = side;
    rclcpp::CallbackGroup::SharedPtr sub_cbg_ = nh_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    if (side_ == 'l') {
        if (!nh_->has_parameter("left_topic"))
            RCLCPP_ERROR_STREAM(nh_->get_logger(), nh_->get_name() << ": left_topic not specified");
        topic = nh_->get_parameter("left_topic").as_string();
        offset_ = nh_->get_parameter("l_offset").get_value<float>();
        gradient_ = nh_->get_parameter("l_gradient").get_value<float>();

    } else if (side_ == 'r') {
        if (!nh_->has_parameter("right_topic"))
            RCLCPP_ERROR_STREAM(nh_->get_logger(), nh_->get_name() << ": right_topic not specified");
        topic = nh_->get_parameter("right_topic").as_string();
        offset_ = nh_->get_parameter("r_offset").get_value<float>();
        gradient_ = nh_->get_parameter("r_gradient").get_value<float>();
    }

    rclcpp::SubscriptionOptions options;
    rclcpp::QoS qos(0);
    options.callback_group = sub_cbg_;
    sub_ = nh_->create_subscription<bitbots_msgs::msg::FloatStamped>(topic + "/raw", qos, std::bind(&HallConverter::hallCb, this, _1), options);
    pub_ = nh_->create_publisher<bitbots_msgs::msg::FloatStamped>(topic + "/filtered", 1);

    sub_executor_.add_callback_group(sub_cbg_, nh_->get_node_base_interface());
    sub_executor_thread_ = new std::thread(
        [this]() {
            sub_executor_.spin();
        });
}

void HallConverter::hallCb(bitbots_msgs::msg::FloatStamped raw) {
  double value = raw.value + offset_; // add offset
  value = (1088 -  value) * gradient_;
  if(value < -M_PI) { // wrap around
    value = value + 2 * M_PI;
  }
  else if(value > M_PI) {
    value = value - 2 * M_PI;
  }
  bitbots_msgs::msg::FloatStamped msg;
  msg.value = value;
  msg.header.stamp = nh_->get_clock()->now();
  pub_->publish(msg);
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // declare parameters automatically
  rclcpp::NodeOptions options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("hall_converter", options);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(nh);

  HallConverter r(nh, 'r');
  HallConverter l(nh, 'l');

  executor.spin();

  return 0;
}
