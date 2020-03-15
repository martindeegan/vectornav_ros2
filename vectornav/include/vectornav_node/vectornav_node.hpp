#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <vn/sensors.h>
#include <vn/thread.h>

namespace vn_ros {

class VectorNavNode : public rclcpp::Node {
  public:
    explicit VectorNavNode(const rclcpp::NodeOptions& options);
    ~VectorNavNode();

  private:
    static void vncxx_callback(void* user_data, vn::protocol::uart::Packet& packet, size_t index);
    void        read_imu(vn::protocol::uart::Packet& packet, size_t index);

    void check_health();

    vn::sensors::VnSensor sensor_;

    sensor_msgs::msg::Imu                               imu_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr                        timer_;
    size_t                                              samples_read;
};

} // namespace vn_ros