#include <vectornav_node/vectornav_node.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions                       options;
    rclcpp::executors::SingleThreadedExecutor exec;

    vn_ros::VectornavNode::SharedPtr node = std::make_shared<vn_ros::VectornavNode>(options);
    exec.add_node(node);

    exec.spin();

    rclcpp::shutdown();

    return 0;
}