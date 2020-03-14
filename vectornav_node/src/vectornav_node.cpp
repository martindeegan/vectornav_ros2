#include <vn/searcher.h>

#include <vectornav_node/vectornav_node.hpp>

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
    timer_           = create_wall_timer(sample_period, std::bind(&VectornavNode::read_imu, this));
    RCLCPP_INFO(get_logger(), "Sampling period: %f", sample_period.count());
}

VectornavNode::~VectornavNode() {
    sensor_.disconnect();
}

void VectornavNode::read_imu() {
    imu_msg_.header.stamp = rclcpp::Time();

    const auto acceleration        = sensor_.readAccelerationMeasurements();
    imu_msg_.linear_acceleration.x = acceleration.x;
    imu_msg_.linear_acceleration.y = acceleration.y;
    imu_msg_.linear_acceleration.z = acceleration.z;

    const auto angular_rates    = sensor_.readAngularRateMeasurements();
    imu_msg_.angular_velocity.x = angular_rates.x;
    imu_msg_.angular_velocity.y = angular_rates.y;
    imu_msg_.angular_velocity.z = angular_rates.z;

    publisher_->publish(imu_msg_);
}

} // namespace vn_ros