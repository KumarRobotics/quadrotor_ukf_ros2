#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"  
#include "sensor_msgs/msg/imu.hpp"  
#include "geometry_msgs/msg/pose_stamped.hpp"  
#include "geometry_msgs/msg/transform_stamped.hpp"  
                                                    
#include "tf2/LinearMath/Quaternion.h"
// #include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include <tf2_eigen/tf2_eigen.hpp>
                                               
#include "quadrotor_ukf_ros2/quadrotor_ukf.h"
#include "quadrotor_ukf_ros2/vio_utils.h"

#include <memory>

class QuadrotorUKFNode : public rclcpp::Node
{
public:
    QuadrotorUKFNode()
    : Node("quadrotor_ukf_ros2")
    {
        // Quadrotor UKF Initialization
        H_C_B_(0,0) = 1;
        H_C_B_(0,1) = 0;
        H_C_B_(0,2) = 0;
        H_C_B_(0,3) = 0;
        H_C_B_(1,0) = 0;
        H_C_B_(1,1) = -1;
        H_C_B_(1,2) = 0;
        H_C_B_(1,3) = 0;
        H_C_B_(2,0) = 0;
        H_C_B_(2,1) = 0;
        H_C_B_(2,2) = -1;
        H_C_B_(2,3) = 0.0;
        H_C_B_(3,0) = 0;
        H_C_B_(3,1) = 0;
        H_C_B_(3,2) = 0;
        H_C_B_(3,3) = 1;

        calLimit_ = 100;
        calCnt_   = 0;

        double alpha;
        double beta;
        double kappa;
        double stdAcc[3]     = {0,0,0};
        double stdW[3]       = {0,0,0};
        double stdAccBias[3] = {0,0,0};

        // Parameter Declaration
        this->declare_parameter<std::string>("odom", std::string("odom"));
        this->declare_parameter<std::string>("imu", std::string("imu"));
        this->declare_parameter<std::string>("base_link", std::string("base_link"));
        this->declare_parameter<std::string>("base_link_frd", std::string("base_link_frd"));
        this->declare_parameter<std::string>("imu_rotated_frame_id", std::string("imu_rotated_base"));
        this->declare_parameter<double>("alpha", 0.4);
        this->declare_parameter<double>("beta" , 2.0);
        this->declare_parameter<double>("kappa", 0.0);
        this->declare_parameter<double>("noise_std/process/acc/x", 0.2);
        this->declare_parameter<double>("noise_std/process/acc/y", 0.2);
        this->declare_parameter<double>("noise_std/process/acc/z", 0.2);
        this->declare_parameter<double>("noise_std/process/w/x", 0.1);
        this->declare_parameter<double>("noise_std/process/w/y", 0.1);
        this->declare_parameter<double>("noise_std/process/w/z", 0.1);
        this->declare_parameter<double>("noise_std/process/acc_bias/x", 0.05);
        this->declare_parameter<double>("noise_std/process/acc_bias/y", 0.05);
        this->declare_parameter<double>("noise_std/process/acc_bias/z", 0.05);

        // Parameter Retrieval
        this->get_parameter("odom", odom_frame_id_);
        this->get_parameter("imu", imu_frame_id_);
        this->get_parameter("base_link", body_frame_id_);
        this->get_parameter("base_link_frd", body_local_frame_id_);
        this->get_parameter("imu_rotated_frame_id", imu_rotated_base_frame_id_);
        this->get_parameter("alpha", alpha);
        this->get_parameter("beta" , beta );
        this->get_parameter("kappa", kappa);
        this->get_parameter("noise_std/process/acc/x", stdAcc[0]);
        this->get_parameter("noise_std/process/acc/y", stdAcc[1]);
        this->get_parameter("noise_std/process/acc/z", stdAcc[2]);
        this->get_parameter("noise_std/process/w/x", stdW[0]);
        this->get_parameter("noise_std/process/w/y", stdW[1]);
        this->get_parameter("noise_std/process/w/z", stdW[2]);
        this->get_parameter("noise_std/process/acc_bias/x", stdAccBias[0]);
        this->get_parameter("noise_std/process/acc_bias/y", stdAccBias[1]);
        this->get_parameter("noise_std/process/acc_bias/z", stdAccBias[2]);

        // Fixed process noise
        Eigen::Matrix<double, 9, 9> Rv;
        Rv.setIdentity();// = eye<mat>(9,9);
        Rv(0,0)   = stdAcc[0] * stdAcc[0];
        Rv(1,1)   = stdAcc[1] * stdAcc[1];
        Rv(2,2)   = stdAcc[2] * stdAcc[2];
        Rv(3,3)   = stdW[0] * stdW[0];
        Rv(4,4)   = stdW[1] * stdW[1];
        Rv(5,5)   = stdW[2] * stdW[2];
        Rv(6,6)   = stdAccBias[0] * stdAccBias[0];
        Rv(7,7)   = stdAccBias[1] * stdAccBias[1];
        Rv(8,8)   = stdAccBias[2] * stdAccBias[2];

        // Initialize UKF
        quadrotorUKF_.SetUKFParameters(alpha, beta, kappa);
        quadrotorUKF_.SetImuCovariance(Rv);

        // ROS2-Specific Initialization:
        // tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // qos profile to match voxl-mpa-to-ros2
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));  // History policy
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);  // Reliability policy
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);  // Durability policy
                                                                     
        // TF members for imu to body offsets
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Define the subscriptions to topics
        // qvio/odometry for QuadrotorUKF Update
        vio_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos_profile, std::bind(&QuadrotorUKFNode::vio_callback, this, std::placeholders::_1));

        // /imu_apps for QuadrotorUKF Prediction
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", qos_profile, std::bind(&QuadrotorUKFNode::imu_callback, this, std::placeholders::_1));

        // /qvio/pose for TF Correction
        // pose_to_tf_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     "/qvio/pose", qos_profile, std::bind(&QuadrotorUKFNode::pose_to_tf_callback, this, std::placeholders::_1));

        // output goal is to produce accurate odom at 300Hz on RELEASE build
        ukf_publisher = this->create_publisher<nav_msgs::msg::Odometry>("control_odom", 10);

        init();

    }

    void init();


private:
    void imu_callback(const sensor_msgs::msg::Imu::UniquePtr msg);
    void vio_callback(const nav_msgs::msg::Odometry::UniquePtr msg);
    // void pose_to_tf_callback(const geometry_msgs::msg::PoseStamped::UniquePtr pose);

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_to_tf_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ukf_publisher;

    // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    //arma::mat H_C_B = arma::eye<mat>(4,4);//Never use reshape
    Eigen::Matrix<double, 4, 4> H_C_B_;
    Eigen::Matrix<double, 4, 4> H_I_B_;

    QuadrotorUKF quadrotorUKF_;
    std::string odom_frame_id_;
    std::string frame_id;

    int calLimit_;
    int calCnt_;
    Eigen::Matrix<double, 3, 1> average_g_;

    std::string imu_frame_id_, imu_rotated_base_frame_id_, body_frame_id_, body_local_frame_id_;
    bool tf_initialized_;
    geometry_msgs::msg::TransformStamped tf_imu_to_base_;
};

void QuadrotorUKFNode::init()
{
  // ros::Rate rate(2.0);
  // while (nh_.ok() && !tf_initialized_)
  // {
  //   try
  //   {
  //     tf_imu_to_base_ = tfBuffer_.lookupTransform(body_frame_id_, imu_frame_id_, ros::Time(0));

  //     //Get the rotation matrix and compose Homogenous matrix (without translation)
  //     Eigen::Affine3d R_I_B = tf2::transformToEigen(tf_imu_to_base_);
  //     H_I_B_.block(0,0,3,3) = R_I_B.rotation();
  //     H_I_B_(3,3) = 1;


  //     ROS_DEBUG_STREAM("Got imu to imu_rotated_base tf " << tf_imu_to_base_);
  //     ROS_DEBUG_STREAM("H_I_B\n" << H_I_B_);

  //     tf_initialized_ = true;
  //   }
  //   catch (tf2::TransformException &ex)
  //   {
  //     ROS_WARN_THROTTLE(1, "Failed to find transform from [%s] to [%s]",
  //                       imu_frame_id_.c_str(), imu_rotated_base_frame_id_.c_str());
  //   }
  // }

  rclcpp::Rate rate(2);
  while(rclcpp::ok() && !tf_initialized_) {
    try {
      tf_imu_to_base_ = tf_buffer_->lookupTransform(body_frame_id_, imu_frame_id_, tf2::TimePointZero);
      //Get the rotation matrix and compose Homogenous matrix (without translation)
      Eigen::Affine3d R_I_B = tf2::transformToEigen(tf_imu_to_base_);
      H_I_B_.block(0,0,3,3) = R_I_B.rotation();
      H_I_B_(3,3) = 1;
      tf_initialized_ = true;

    } 
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to get TF!");
    }
    rate.sleep();

  }

  // tfListener_.reset();
  RCLCPP_INFO(this->get_logger(), "Quadrotor UKF Initialized");
}

// Callback for IMU data
void QuadrotorUKFNode::imu_callback(const sensor_msgs::msg::Imu::UniquePtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received IMU data");
    static Eigen::Matrix<double, 3, 1> ag;
    Eigen::Matrix<double, 6, 1> u;
    u(0,0) = msg->linear_acceleration.x;
    u(1,0) = -msg->linear_acceleration.y;
    u(2,0) = -msg->linear_acceleration.z;
    u(3,0) = msg->angular_velocity.x;
    u(4,0) = -msg->angular_velocity.y;
    u(5,0) = -msg->angular_velocity.z;
    if (calCnt_ < calLimit_)       // Calibration
    {
      calCnt_++;
      ag += u.block(0,0,3,1);//rows(0,2);
    }
    else if (calCnt_ == calLimit_) // Save gravity vector
    {
      calCnt_++;
      ag /= calLimit_;
      double g = ag.norm();//norm(ag,2);
      quadrotorUKF_.SetGravity(g);
      RCLCPP_INFO(this->get_logger(), "Calibration Complete g: %f", g);
    }
    else if (quadrotorUKF_.ProcessUpdate(u, msg->header.stamp))  // Process Update
    {
      nav_msgs::msg::Odometry odomUKF;
      // Publish odom
      odomUKF.header.stamp = quadrotorUKF_.GetStateTime();
      odomUKF.header.frame_id = frame_id;
      Eigen::Matrix<double, Eigen::Dynamic, 1> x = quadrotorUKF_.GetState();
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
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  P = quadrotorUKF_.GetStateCovariance();
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
// void QuadrotorUKFNode::pose_to_tf_callback(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
// {
//   // TODO: GRAB FRAME ID FROM VOXL-MPA-TO-ROS2
//     RCLCPP_INFO(this->get_logger(), "Received pose to TF data");
//     // TODO: Convert pose to TF and publish it
//     static geometry_msgs::msg::TransformStamped static_transformStamped;
//     static_transformStamped.header.stamp = this->get_clock()->now();
//     static_transformStamped.header.frame_id = "map_ned";
//     static_transformStamped.child_frame_id = "base_link_frd";
// 
//     static_transformStamped.transform.translation.x = msg->pose.position.x;
//     static_transformStamped.transform.translation.y = msg->pose.position.y;
//     static_transformStamped.transform.translation.z = msg->pose.position.z;
// 
//     static_transformStamped.transform.rotation.x = msg->pose.orientation.x;
//     static_transformStamped.transform.rotation.y = msg->pose.orientation.y;
//     static_transformStamped.transform.rotation.z = msg->pose.orientation.z;
//     static_transformStamped.transform.rotation.w = msg->pose.orientation.w;
// 
// 
//     tf_broadcaster_->sendTransform(static_transformStamped);
// }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadrotorUKFNode>());
    rclcpp::shutdown();
    return 0;
}

