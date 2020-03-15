#include <rclcpp/rclcpp.hpp>
#include <vectornav_node/vectornav_node.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    vn_ros::VectorNavNode::SharedPtr node = std::make_shared<vn_ros::VectorNavNode>(options);

    while (rclcpp::ok()) {
        rclcpp::sleep_for(1s);
    }
    rclcpp::shutdown();

    return 0;
}