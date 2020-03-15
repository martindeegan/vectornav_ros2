#include <functional>

#include <vn/searcher.h>

#include <vectornav/vectornav_node.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

using vn::protocol::uart::AsciiAsync;
using vn::protocol::uart::Packet;

namespace vn_ros {

VectorNavNode::VectorNavNode(const rclcpp::NodeOptions& options)
    : Node("vectornav_node", options), samples_read(0) {
    declare_parameter<std::string>("sensor_port", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 921600);
    declare_parameter<int>("sample_rate", 200);
    declare_parameter<std::string>("topic", "/imu");
    declare_parameter<std::string>("frame_id", "imu");
    declare_parameter<double>("gyroscope_variance", 1e-3);
    declare_parameter<double>("accelerometer_variance", 1e-3);
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

    const auto topic = parameters_client->get_parameter<std::string>("topic");
    publisher_       = create_publisher<sensor_msgs::msg::Imu>(topic, 10);

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

    // Use vncxx to subscribe to the async stream of data
    sensor_.writeAsyncDataOutputType(AsciiAsync::VNQMR);
    sensor_.registerAsyncPacketReceivedHandler(this, &VectorNavNode::vncxx_callback);
}

VectorNavNode::~VectorNavNode() {
    sensor_.disconnect();
}

void VectorNavNode::vncxx_callback(void* user_data, Packet& packet, size_t index) {
    auto node = reinterpret_cast<vn_ros::VectorNavNode*>(user_data);
    node->read_imu(packet, index);
}

void VectorNavNode::read_imu(Packet& packet, size_t index) {
    imu_msg_.header.stamp = rclcpp::Time();

    if (packet.type() != Packet::TYPE_ASCII) {
        RCLCPP_ERROR(get_logger(), "Did not receive ASCII packet.");
        return;
    }

    if (packet.determineAsciiAsyncType() != AsciiAsync::VNQMR) {
        RCLCPP_ERROR(get_logger(), "Did not receive correct ASCII type.");
        return;
    }

    vn::math::vec4f quaternion;
    vn::math::vec3f magnetic_field;
    vn::math::vec3f angular_rates;
    vn::math::vec3f linear_acceleration;
    packet.parseVNQMR(&quaternion, &magnetic_field, &linear_acceleration, &angular_rates);

    imu_msg_.angular_velocity.x = angular_rates.x;
    imu_msg_.angular_velocity.y = angular_rates.y;
    imu_msg_.angular_velocity.z = angular_rates.z;

    imu_msg_.linear_acceleration.x = linear_acceleration.x;
    imu_msg_.linear_acceleration.y = linear_acceleration.y;
    imu_msg_.linear_acceleration.z = linear_acceleration.z;

    imu_msg_.orientation.w = quaternion.w;
    imu_msg_.orientation.x = quaternion.x;
    imu_msg_.orientation.y = quaternion.y;
    imu_msg_.orientation.z = quaternion.z;

    publisher_->publish(imu_msg_);
    samples_read++;
}

} // namespace vn_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vn_ros::VectorNavNode)
