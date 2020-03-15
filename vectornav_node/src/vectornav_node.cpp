#include <functional>

#include <vn/searcher.h>

#include <vectornav_node/vectornav_node.hpp>

using namespace std::placeholders;

using vn::protocol::uart::AsciiAsync;
using vn::protocol::uart::Packet;

namespace vn_ros {

VectornavNode::VectornavNode(const rclcpp::NodeOptions& options)
    : Node("vectornav_node", options), samples_read(0) {
    declare_parameter<std::string>("sensor_port", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 921600);
    declare_parameter<int>("sample_rate", 200);
    declare_parameter<std::string>("topic", "/imu");
    declare_parameter<std::string>("frame_id", "imu");
    declare_parameter<double>("gyroscope_variance", 1e-3);
    declare_parameter<double>("accelerometer_variance", 1e-3);
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

    // Find the IMU port and baudrate
    const auto valid_ports = vn::sensors::Searcher::search();
    if (valid_ports.empty()) {
        RCLCPP_ERROR(get_logger(), "VectorNav IMU not found on any valid ports.");
        rclcpp::shutdown();
    }

    const auto sensor_port = valid_ports[0].first;
    const auto baudrate    = valid_ports[0].second;
    sensor_.connect(sensor_port, baudrate);

    if (!sensor_.isConnected()) {
        RCLCPP_ERROR(get_logger(), "VectorNav IMU not connected.");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(get_logger(), "Model Number: %s", sensor_.readModelNumber().c_str());
    RCLCPP_INFO(get_logger(), "Serial Number: %i", sensor_.readSerialNumber());
    RCLCPP_INFO(get_logger(), "Hardware Revision Number: %i", sensor_.readHardwareRevision());
    RCLCPP_INFO(get_logger(), "Firmware Version: %s", sensor_.readFirmwareVersion().c_str());

    // Set the user desired baudrate
    const auto desired_baudrate = parameters_client->get_parameter<int>("baudrate");
    if (baudrate != desired_baudrate) {
        sensor_.changeBaudRate(desired_baudrate);
    }

    RCLCPP_INFO(get_logger(), "Baudrate: %i", sensor_.baudrate());

    const auto sample_rate   = parameters_client->get_parameter<int>("sample_rate");
    const auto sample_period = std::chrono::duration<double>(1) / static_cast<double>(sample_rate);
    sensor_.writeAsyncDataOutputFrequency(sample_rate);
    RCLCPP_INFO(get_logger(), "Sampling frequency: %i Hz", sensor_.readAsyncDataOutputFrequency());

    // Construct parts of the imu message
    imu_msg_.header.frame_id = parameters_client->get_parameter<std::string>("frame_id");

    const auto gyroscope_variance = parameters_client->get_parameter<double>("gyroscope_variance");
    const auto accelerometer_variance =
        parameters_client->get_parameter<double>("accelerometer_variance");

    imu_msg_.linear_acceleration_covariance[0] = accelerometer_variance;
    imu_msg_.linear_acceleration_covariance[4] = accelerometer_variance;
    imu_msg_.linear_acceleration_covariance[8] = accelerometer_variance;

    imu_msg_.angular_velocity_covariance[0] = gyroscope_variance;
    imu_msg_.angular_velocity_covariance[4] = gyroscope_variance;
    imu_msg_.angular_velocity_covariance[8] = gyroscope_variance;

    // Create publisher and start sampling timer
    const auto topic = parameters_client->get_parameter<std::string>("topic");
    publisher_       = create_publisher<sensor_msgs::msg::Imu>(topic, 10);

    sensor_.writeAsyncDataOutputType(AsciiAsync::VNQMR);
    sensor_.registerAsyncPacketReceivedHandler(this, &VectornavNode::vncxx_callback);
}

VectornavNode::~VectornavNode() {
    sensor_.disconnect();
}

void VectornavNode::vncxx_callback(void* user_data, Packet& packet, size_t index) {
    auto node = reinterpret_cast<vn_ros::VectornavNode*>(user_data);
    node->read_imu(packet, index);
}

void VectornavNode::read_imu(Packet& packet, size_t index) {
    imu_msg_.header.stamp = rclcpp::Time();

    if (packet.type() != Packet::TYPE_ASCII) {
        RCLCPP_ERROR(get_logger(), "Did not receive ASCII packet.");
        return;
    }

    if (packet.determineAsciiAsyncType() != AsciiAsync::VNQMR) {
        RCLCPP_ERROR(get_logger(), "Did not receive correct ASCII type.");
        return;
    }

    vn::sensors::QuaternionMagneticAccelerationAndAngularRatesRegister reg;
    packet.parseQuaternionMagneticAccelerationAndAngularRates(
        &reg.quat, &reg.mag, &reg.accel, &reg.gyro);

    imu_msg_.angular_velocity.x = reg.gyro.x;
    imu_msg_.angular_velocity.y = reg.gyro.y;
    imu_msg_.angular_velocity.z = reg.gyro.z;

    imu_msg_.linear_acceleration.x = reg.accel.x;
    imu_msg_.linear_acceleration.y = reg.accel.y;
    imu_msg_.linear_acceleration.z = reg.accel.z;

    imu_msg_.orientation.w = reg.quat.w;
    imu_msg_.orientation.x = reg.quat.x;
    imu_msg_.orientation.y = reg.quat.y;
    imu_msg_.orientation.z = reg.quat.z;

    publisher_->publish(imu_msg_);
    samples_read++;
}

} // namespace vn_ros