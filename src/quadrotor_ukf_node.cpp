#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"  // Change this to the actual message type you'll use for each callback
#include "sensor_msgs/msg/imu.hpp"  // Change this to the actual message type you'll use for each callback
#include "geometry_msgs/msg/pose_stamped.hpp"  // Change this to the actual message type you'll use for each callback
                                               //
#include "quadrotor_ukf_ros2/quadrotor_ukf.h"
#include "quadrotor_ukf_ros2/vio_utils.h"

static QuadrotorUKF quadrotorUKF;
static std::string frame_id;

class QuadrotorUKFNode : public rclcpp::Node
{
public:
    QuadrotorUKFNode()
    : Node("quadrotor_ukf_ros2")
    {
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));  // History policy
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);  // Reliability policy
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);  // Durability policy

        // Define the subscriptions to topics
        vio_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/qvio/odometry", qos_profile, std::bind(&QuadrotorUKFNode::vio_callback, this, std::placeholders::_1));

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_apps", qos_profile, std::bind(&QuadrotorUKFNode::imu_callback, this, std::placeholders::_1));

        pose_to_tf_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/qvio/pose", qos_profile, std::bind(&QuadrotorUKFNode::pose_to_tf_callback, this, std::placeholders::_1));

        ukf_publisher = this->create_publisher<nav_msgs::msg::Odometry>("control_odom", 10);
    }
    void imu_callback(const sensor_msgs::msg::Imu::UniquePtr msg);
    void vio_callback(const nav_msgs::msg::Odometry::UniquePtr msg);
    void pose_to_tf_callback(const geometry_msgs::msg::PoseStamped::UniquePtr pose);

private:

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_to_tf_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ukf_publisher;
};

// Callback for IMU data
void QuadrotorUKFNode::imu_callback(const sensor_msgs::msg::Imu::UniquePtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received IMU data");
    static int calLimit = 100;
    static int calCnt   = 0;
    static Eigen::Matrix<double, 3, 1> ag;
    Eigen::Matrix<double, 6, 1> u;
    u(0,0) = msg->linear_acceleration.x;
    u(1,0) = -msg->linear_acceleration.y;
    u(2,0) = -msg->linear_acceleration.z;
    u(3,0) = msg->angular_velocity.x;
    u(4,0) = -msg->angular_velocity.y;
    u(5,0) = -msg->angular_velocity.z;
    if (calCnt < calLimit)       // Calibration
    {
      calCnt++;
      ag += u.block(0,0,3,1);//rows(0,2);
    }
    else if (calCnt == calLimit) // Save gravity vector
    {
      calCnt++;
      ag /= calLimit;
      double g = ag.norm();//norm(ag,2);
      quadrotorUKF.SetGravity(g);
      RCLCPP_INFO(this->get_logger(), "Calibration Complete g: %f", g);
    }
    else if (quadrotorUKF.ProcessUpdate(u, msg->header.stamp))  // Process Update
    {
      nav_msgs::msg::Odometry odomUKF;
      // Publish odom
      odomUKF.header.stamp = quadrotorUKF.GetStateTime();
      odomUKF.header.frame_id = frame_id;
      Eigen::Matrix<double, Eigen::Dynamic, 1> x = quadrotorUKF.GetState();
      odomUKF.pose.pose.position.x = x(0,0);
      odomUKF.pose.pose.position.y = x(1,0);
      odomUKF.pose.pose.position.z = x(2,0);
      Eigen::Matrix<double, 4, 1> q = VIOUtil::MatToQuat(VIOUtil::ypr_to_R(x.block(6,0,3,1)));
      odomUKF.pose.pose.orientation.w = q(0,0);
      odomUKF.pose.pose.orientation.x = q(1,0);
      odomUKF.pose.pose.orientation.y = q(2,0);
      odomUKF.pose.pose.orientation.z = q(3,0);
      odomUKF.twist.twist.linear.x = x(3,0);
      odomUKF.twist.twist.linear.y = x(4,0);
      odomUKF.twist.twist.linear.z = x(5,0);
      odomUKF.twist.twist.angular.x = u(3,0);
      odomUKF.twist.twist.angular.y = u(4,0);
      odomUKF.twist.twist.angular.z = u(5,0);
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  P = quadrotorUKF.GetStateCovariance();
      for (int j = 0; j < 6; j++)
        for (int i = 0; i < 6; i++)
          odomUKF.pose.covariance[i+j*6] = P((i<3)?i:i+3 , (j<3)?j:j+3);
      for (int j = 0; j < 3; j++)
        for (int i = 0; i < 3; i++)
          odomUKF.twist.covariance[i+j*6] = P(i+3 , j+3);
      ukf_publisher->publish(odomUKF);
    }   
}

// Callback for VIO data
void QuadrotorUKFNode::vio_callback(const nav_msgs::msg::Odometry::UniquePtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received VIO data");
    // TODO: Add VIO data processing for UKF
}

// Callback for pose to TF data
void QuadrotorUKFNode::pose_to_tf_callback(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received pose to TF data");
    // TODO: Convert pose to TF and publish it
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadrotorUKFNode>());
    rclcpp::shutdown();
    return 0;
}

