#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"  // Change this to the actual message type you'll use for each callback
#include "nav_msgs/msg/odometry.hpp"  // Change this to the actual message type you'll use for each callback
#include "geometry_msgs/msg/posestamped.hpp"  // Change this to the actual message type you'll use for each callback

class QuadrotorUKFNode : public rclcpp::Node
{
public:
    QuadrotorUKFNode()
    : Node("quadrotor_ukf_ros2")
    {
        // Define the subscriptions to topics
        vio_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/qvio/odometry", 10, std::bind(&QuadrotorUKFNode::vio_callback, this, std::placeholders::_1));

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_apps", 10, std::bind(&QuadrotorUKFNode::imu_callback, this, std::placeholders::_1));

        pose_to_tf_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/qvio/pose", 10, std::bind(&QuadrotorUKFNode::pose_to_tf_callback, this, std::placeholders::_1));
    }

private:
    // Callback for VIO data
    void vio_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received VIO data");
        
        // TODO: Add VIO data processing for UKF
    }

    // Callback for IMU data
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received IMU data");
        // TODO: Add IMU data processing for UKF
    }

    // Callback for pose to TF data
    void pose_to_tf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received pose to TF data");
        // TODO: Convert pose to TF and publish it
    }

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_to_tf_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadrotorUKFNode>());
    rclcpp::shutdown();
    return 0;
}

