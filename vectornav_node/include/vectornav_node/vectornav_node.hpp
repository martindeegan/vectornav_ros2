#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <vn/sensors.h>
#include <vn/thread.h>

namespace vn_ros {

class VectornavNode : public rclcpp::Node {
  public:
    explicit VectornavNode(const rclcpp::NodeOptions& options);
    ~VectornavNode();

  private:
    void read_imu();

    vn::sensors::VnSensor sensor_;

    sensor_msgs::msg::Imu                               imu_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr                        timer_;
    size_t                                              samples_read;
};

} // namespace vn_ros