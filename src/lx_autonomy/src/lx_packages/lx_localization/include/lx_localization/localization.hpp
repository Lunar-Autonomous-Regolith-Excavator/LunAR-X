#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.h>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "lx_msgs/action/calibrate_imu.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lx_msgs/msg/rover_command.hpp"
#include <Eigen/Dense>
#include <visualization_msgs/msg/marker.hpp>

class Localization : public rclcpp::Node
{
    private:
        bool DEBUG_ = false;
        // Variables & pointers -----------------
        using CalibrateImu = lx_msgs::action::CalibrateImu;
        using GoalHandleCalibrateImu = rclcpp_action::ServerGoalHandle<CalibrateImu>;
        geometry_msgs::msg::Quaternion imu_orientation_;
        geometry_msgs::msg::Point ts_point_;
        bool got_imu_ = false;
        bool got_transforms_ = false;
        bool printed_all_working_ = false, printed_calibration_not_complete_ = false;
        bool calibration_complete_ = false;
        double yaw_offset_ = 0.0; // Add this value to IMU yaw to get total station yaw
        // Time
        rclcpp::Time last_imu_msg_time_ = rclcpp::Time(0,0, RCL_ROS_TIME);
        rclcpp::Time last_ts_msg_time_ = rclcpp::Time(0,0, RCL_ROS_TIME);
        // Transforms
        geometry_msgs::msg::TransformStamped eigen_transform_prism_baselink_;
        geometry_msgs::msg::TransformStamped eigen_transform_imu_baselink_;
        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<lx_msgs::msg::RoverCommand>::SharedPtr rover_command_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        // Actions
        rclcpp_action::Server<CalibrateImu>::SharedPtr calibrate_imu_action_server_;
        // --------------------------------------


        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        *
        * */
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CalibrateImu::Goal> );

        /*
        *
        * */
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCalibrateImu> );

        /*
        *
        * */
        void handle_accepted(const std::shared_ptr<GoalHandleCalibrateImu> );

        /*
        *
        * */
        void executeCalibrateIMU(const std::shared_ptr<GoalHandleCalibrateImu> );

        /*
        *
        * */
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr );

        /*
        *
        * */
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr );

        /*
        *
        * */
        bool get_transforms();

        /*
        *
        * */
        void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr );

    public:
        // Functions
        /*
        * Constructor
        * */
        Localization();

        /*
        * Destructor
        * */
        ~Localization(){};

};

#endif