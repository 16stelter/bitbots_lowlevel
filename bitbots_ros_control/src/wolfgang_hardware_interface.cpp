#include <bitbots_ros_control/wolfgang_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {

/**
 * This class provides a combination of multiple hardware interfaces to construct a complete Wolfgang robot.
 * It is similar to a CombinedRobotHw class as specified in ros_control, but it is changed a bit to make sharing of
 * a common bus driver over multiple hardware interfaces possible.
 */
WolfgangHardwareInterface::WolfgangHardwareInterface(ros::NodeHandle &nh) {
  speak_pub_ = nh.advertise<humanoid_league_msgs::Speak>("/speak", 1);

  // load parameters
  ROS_INFO_STREAM("Loading parameters from namespace " << nh.getNamespace());
  nh.getParam("only_imu", only_imu_);
  if (only_imu_) ROS_WARN("Starting in only IMU mode");
  nh.getParam("only_pressure", only_pressure_);
  if (only_pressure_) ROS_WARN("starting in only pressure sensor mode");

  if (only_pressure_ && only_imu_) {
    ROS_ERROR("only_imu AND only_pressure was set to true");
    exit(1);
  }

  // get list of all bus devices
  XmlRpc::XmlRpcValue dxls_xml;
  nh.getParam("device_info", dxls_xml);
  ROS_ASSERT(dxls_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  // Convert dxls to native type: a vector of tuples with name and id for sorting purposes
  std::vector<std::pair<std::string, int>> dxl_devices;
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = dxls_xml.begin(); it != dxls_xml.end(); ++it) {
    std::string name = it->first;
    XmlRpc::XmlRpcValue data = it->second;
    int id = data["id"];
    dxl_devices.emplace_back(name, id);
  }

  // sort the devices by id. This way the devices will always be read and written in ID order later, making debug easier.
  std::sort(dxl_devices.begin(), dxl_devices.end(),
            [](std::pair<std::string, int> &a, std::pair<std::string, int> &b) { return a.second < b.second; });


  // init bus drivers
  XmlRpc::XmlRpcValue port_xml;
  nh.getParam("port_info", port_xml);
  int devices_found = 0;
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = port_xml.begin(); it != port_xml.end(); ++it) {
    // read bus driver specifications from config
    XmlRpc::XmlRpcValue port_data = it->second;
    std::string device_file = port_data["device_file"];
    int baudrate = port_data["baudrate"];
    int protocol_version = port_data["protocol_version"];
    auto driver = std::make_shared<DynamixelDriver>();
    if (!driver->init(device_file.c_str(), uint32_t(baudrate))) {
      ROS_ERROR("Error opening serial port %s", device_file.c_str());
      speakError(speak_pub_, "Error opening serial port");
      sleep(1);
      exit(1);
    }
    driver->setPacketHandler(protocol_version);
    std::vector<hardware_interface::RobotHW> interfaces_on_port;
    // iterate over all devices and ping them to see what is connected to this bus
    std::vector<std::tuple<int, std::string, float, float>> servos_on_port;
    for (std::pair<std::string, int> &device : dxl_devices) {
      std::string name = device.first;
      int id = device.second;
      ros::NodeHandle dxl_nh(nh, "device_info/" + name);
      int model_number_specified;
      dxl_nh.getParam("model_number", model_number_specified);
      uint16_t model_number_specified_16 = uint16_t(model_number_specified);
      uint16_t *model_number_returned_16;
      if (driver->ping(uint8_t(id), model_number_returned_16)) {
        // check if the specified model number matches the actual model number of the device
        if (model_number_specified_16 != *model_number_returned_16) {
          ROS_WARN("Model number of id %d does not match", id);
        }
        // ping was successful, add device correspondingly
        if (model_number_specified == 99) {//todo correct number
          std::string topic;
          dxl_nh.getParam("topic", topic);
          interfaces_on_port.push_back(BitFootHardwareInterface(driver, id, topic));
        } else if (model_number_specified == 100) { //todo correct number
          std::string topic;
          dxl_nh.getParam("topic", topic);
          std::string frame;
          dxl_nh.getParam("frame", frame);
          interfaces_on_port.push_back(ImuHardwareInterface(driver, id, topic, frame, name));
        } else if (model_number_specified == 101) { //todo correct number
          std::string topic;
          dxl_nh.getParam("topic", topic);
          interfaces_on_port.push_back(ButtonHardwareInterface(driver, id, topic));
        } else if (model_number_specified == 311 || model_number_specified == 321) {
          float mounting_offset;
          dxl_nh.getParam("mounting_offset", mounting_offset);
          float joint_offset;
          dxl_nh.getParam("joint_offset", joint_offset);
          servos_on_port.push_back(std::make_tuple(id, name, mounting_offset, joint_offset));
        } else {
          ROS_WARN("Could not identify device for ID %d", id);
        }
        devices_found++;
      }
    }
    if (servos_on_port.size() > 0) {
      interfaces_on_port.push_back(DynamixelServoHardwareInterface(driver, servos_on_port));
    }
    // add vector of interfaces on this port to overall collection of interfaces
    interfaces_.push_back(interfaces_on_port);
  }

  if (devices_found != dxl_devices.size()) {
    ROS_WARN("Could not ping all devices!");
  }

  // set the dynamic reconfigure and load standard params for servo interface
  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig> server;
  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig>::CallbackType f;
  f = boost::bind(&bitbots_ros_control::DynamixelServoHardwareInterface::reconfCallback, &servos_, _1, _2);
  server.setCallback(f);

}

bool WolfgangHardwareInterface::init(ros::NodeHandle &root_nh) {
  bool success = true;
  // iterate through all ports
  for (std::vector<hardware_interface::RobotHW> &port_interfaces : interfaces_) {
    // iterate through all interfaces on this port
    for(hardware_interface::RobotHW &interface : port_interfaces){
      if()
    }
  }

  if (only_imu_) {
    imu_.setParent(this);
    success &= imu_.init(root_nh);
  } else if (only_pressure_) {
    success &= left_foot_.init(root_nh);
    success &= right_foot_.init(root_nh);
  } else {
    /* Hardware interfaces must be registered at the main RobotHW class.
     * Therefore, a pointer to this class is passed down to the RobotHW classes
     * registering further interfaces */
    servos_.setParent(this);
    imu_.setParent(this);
    success &= servos_.init(root_nh);
    success &= imu_.init(root_nh);
    success &= left_foot_.init(root_nh);
    success &= right_foot_.init(root_nh);
    success &= buttons_.init(root_nh);
  }
  if (success) {
    speakError(speak_pub_, "ros control startup successful");
  } else {
    speakError(speak_pub_, "error starting ros control");
  }
  return success;
}

bool WolfgangHardwareInterface::read() {
  bool success = true;
  if (only_imu_) {
    success &= imu_.read();
  } else if (only_pressure_) {
    success &= left_foot_.read();
    success &= right_foot_.read();
  } else {
    success &= servos_.read();
    success &= imu_.read();
    success &= left_foot_.read();
    success &= right_foot_.read();
    success &= buttons_.read();
  }
  return success;
}

void WolfgangHardwareInterface::write() {
  if (!only_imu_ && !only_pressure_) {
    servos_.write();
  }
}
}