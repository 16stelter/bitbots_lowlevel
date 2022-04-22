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
        data_ = (uint8_t *) malloc(4 * sizeof(uint8_t));
        return true;
    }

    void HallHardwareInterface::read(const rclcpp::Time &t, const rclcpp::Duration &dt) {
        bool read_successful = true;
        if(driver_->readMultipleRegisters(id_, 34, 4, data_)) {
            //id, start address, length, data. 34 is angle.
            int32_t pres = dxlMakedword(dxlMakeword(data_[0], data_[1]),
                                        dxlMakeword(data_[2], data_[3]));
            float pres_d = (float) pres;
            current_angle_.push_back(pres_d);
            // keep last 10 values to recognize read errors.
        } else {
            RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000, "Could not read %s", name_.c_str());
            read_successful = false;
        }

        msg_.header.stamp = nh_->get_clock()->now();
        msg_.value = current_angle_.back();
        angle_pub_->publish(msg_);

        if (current_angle_.size() > 10) {
            current_angle_.erase(current_angle_.begin()); // delete oldest if more than 10 saved
            diagnostic_msgs::msg::DiagnosticArray array_msg = diagnostic_msgs::msg::DiagnosticArray();
            std::vector<diagnostic_msgs::msg::DiagnosticStatus> array = std::vector<diagnostic_msgs::msg::DiagnosticStatus>();
            array_msg.header.stamp = nh_->get_clock()->now();
            diagnostic_msgs::msg::DiagnosticStatus status = diagnostic_msgs::msg::DiagnosticStatus();

            status.name = "Hall" + name_;
            status.hardware_id = std::to_string(id_);

            bool okay = false;
            for (size_t j = 0; j < current_pangle_.size(); j++) {
                if (current_angle[-1] != current_angle_[j]) {
                    okay = true;
                    break;
                }
            }
            if (read_successful) {
                if (okay) {
                    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                    status.message = "OK";
                }
                else {
                    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                    status.message = "Did not receive new values.";
                }
            }
            else {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                status.message = "Could not read sensor.";
            }
            array.push_back(status);
            array_msg.status = array;
            diagnostic_pub_->publish(array_msg);
        }
    }

    void HallHardwareInterface::write(const rclcpp::Time &t, const rclcpp::Duration &dt) {
        // We don't write anything.
    }
}