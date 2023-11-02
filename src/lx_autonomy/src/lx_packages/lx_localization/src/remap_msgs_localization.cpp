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

class RemapNode : public rclcpp::Node
{
public:
    RemapNode() : Node("remap_msgs_localization")
    {
        // Subscribe to the "odom" topic and remap it to "my_odom"
        auto qos = rclcpp::SensorDataQoS();
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/husky_velocity_controller/odom", qos, std::bind(&RemapNode::odom_callback, this, std::placeholders::_1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/total_station_prism", qos, std::bind(&RemapNode::pose_callback, this, std::placeholders::_1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu >("/vectornav/imu", qos, std::bind(&RemapNode::imu_callback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/husky_odom", qos);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/total_station_pose_map", qos);
        rover_command_pub_ = create_publisher<lx_msgs::msg::RoverCommand>("/rover_auto_cmd", 10);

        calibrate_imu_action_server_ = rclcpp_action::create_server<CalibrateImu>(
            this,
            "lx_localization/calibrate_imu",
            std::bind(&RemapNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RemapNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&RemapNode::handle_accepted, this, std::placeholders::_1));
    }

private:
    using CalibrateImu = lx_msgs::action::CalibrateImu;
    using GoalHandleCalibrateImu = rclcpp_action::ServerGoalHandle<CalibrateImu>;
    
    // Action server
    rclcpp_action::Server<CalibrateImu>::SharedPtr calibrate_imu_action_server_;

    // Transforms
    geometry_msgs::msg::TransformStamped eigen_transform_prism_baselink;
    geometry_msgs::msg::TransformStamped eigen_transform_imu_baselink;

    // Variables
    geometry_msgs::msg::Quaternion imu_orientation;
    geometry_msgs::msg::Point ts_point;
    bool got_imu = false;
    bool got_transforms = false;
    bool printed_all_working = false, printed_calibration_complete = false, printed_calibration_not_complete = false;
    bool calibration_complete = false;
    double yaw_offset = 0.0; // Add this value to IMU yaw to get total station yaw
    // time to store IMU and TS data
    rclcpp::Time last_imu_msg_time = rclcpp::Time(0,0, RCL_ROS_TIME);
    rclcpp::Time last_ts_msg_time = rclcpp::Time(0,0, RCL_ROS_TIME);

    // Subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<lx_msgs::msg::RoverCommand>::SharedPtr rover_command_pub_;
    
    // Action server callbacks template
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CalibrateImu::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received calibrate imu request");
        (void)uuid;
        (void)goal;

        // Accept and execute action
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCalibrateImu> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel calibrate imu action");
        (void)goal_handle;

        // Accept cancel request
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCalibrateImu> goal_handle)
    {
        // Start execution of action
        std::thread{std::bind(&RemapNode::executeCalibrateIMU, this, std::placeholders::_1), goal_handle}.detach();
    }

    void executeCalibrateIMU(const std::shared_ptr<GoalHandleCalibrateImu> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing calibrate imu action");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<CalibrateImu::Feedback>();
        auto result = std::make_shared<CalibrateImu::Result>();

        // Get transforms if we don't have them yet
        if(this->got_transforms == false)
        {
            this->get_transforms();
        }

        // If latest IMU msg and TS msg are more than 1 second away from current time, return error
        auto current_time = this->get_clock()->now();
        if((current_time - this->last_imu_msg_time).seconds() > 1.0 || (current_time - this->last_ts_msg_time).seconds() > 1.0)
        {
            RCLCPP_INFO(this->get_logger(), "No IMU or TS data found, please check if IMU and TS are publishing");
            // Set result
            result->success = false;
            goal_handle->canceled(result);
            return;
        }

        // Get latest IMU yaw in base_link frame and rover pose
        geometry_msgs::msg::Quaternion imu_orientation = this->imu_orientation;
        tf2::doTransform(imu_orientation, imu_orientation, this->eigen_transform_imu_baselink);
        auto yaw_initial = tf2::getYaw(imu_orientation);
        geometry_msgs::msg::Point init_ts_point = this->ts_point;

        // Move the rover ahead for 5 seconds
        auto start_time = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Starting calibration movement");
        while(rclcpp::ok() && !goal_handle->is_canceling())
        {
            // Check if 5 seconds have passed
            auto current_time = this->get_clock()->now();
            auto time_diff = current_time - start_time;
            if(time_diff.seconds() >= 5.0)
            {
                RCLCPP_INFO(this->get_logger(), "Calibration movement complete");
                break;
            }

            // Publish rover command
            lx_msgs::msg::RoverCommand rover_command;
            rover_command.mobility_twist.linear.x = 0.1;
            rover_command_pub_->publish(rover_command);
        }
        if (goal_handle->is_canceling() || !rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), "Calibrate imu action cancelled");
            // Set result
            result->success = false;
            goal_handle->canceled(result);
            return;
        }

        // If latest IMU msg and TS msg are more than 1 second away from current time, return error
        current_time = this->get_clock()->now();
        if((current_time - this->last_imu_msg_time).seconds() > 1.0 || (current_time - this->last_ts_msg_time).seconds() > 1.0)
        {
            RCLCPP_INFO(this->get_logger(), "No IMU or TS data found, please check if IMU and TS are publishing");
            // Set result
            result->success = false;
            goal_handle->canceled(result);
            return;
        }

        // Record final yaw (in base_link frame) and rover pose
        geometry_msgs::msg::Quaternion new_imu_orientation = this->imu_orientation;
        tf2::doTransform(new_imu_orientation, new_imu_orientation, this->eigen_transform_imu_baselink);
        auto yaw_final = tf2::getYaw(new_imu_orientation);
        geometry_msgs::msg::Point final_ts_point = this->ts_point;

        // Calculate yaw offset
        auto avg_imu_yaw = (yaw_initial + yaw_final) / 2.0;
        auto yaw_total_station = atan2(final_ts_point.y - init_ts_point.y, final_ts_point.x - init_ts_point.x);
        this->yaw_offset = yaw_total_station - avg_imu_yaw;

        RCLCPP_INFO(this->get_logger(), "Calibration complete, with yaw offset: %f", this->yaw_offset);
        this->calibration_complete = true;

        // Set result
        result->success = true;
        goal_handle->succeed(result);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update the frame_id field in the message
        msg->header.frame_id = "base_link";
        // reset covariance
        msg->twist.covariance[0] = 0.1; // X
        msg->twist.covariance[7] = 0.01; // y
        msg->twist.covariance[14] = 0.01; // z
        msg->twist.covariance[21] = 0.01; // roll
        msg->twist.covariance[28] = 0.01; // pitch
        msg->twist.covariance[35] = 0.5; // yaw
        odom_pub_->publish(*msg);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        got_imu = true;
        this->imu_orientation = msg->orientation;
        this->last_imu_msg_time = this->get_clock()->now();
    }

    bool get_transforms()
    {
        try 
        {
            std::cout<<"Looking for transform"<<std::endl;
            // Get transform matrix from total_station_prism to base_link
            tf2_ros::Buffer tf_buffer(this->get_clock());
            tf2_ros::TransformListener tf_listener(tf_buffer);
            this->eigen_transform_prism_baselink = tf_buffer.lookupTransform("base_link", "total_station_prism", tf2::TimePointZero, tf2::durationFromSec(0.5)); //prism to base link
            this->eigen_transform_imu_baselink = tf_buffer.lookupTransform("base_link", "vectornav", tf2::TimePointZero, tf2::durationFromSec(0.5)); //primm to base link
            this->got_transforms = true;
            return true;
        } 
        catch (tf2::TransformException& ex) 
        {
            std::cout<<"No transforms found, please check if base_link to total_station_prism and base_link to vectornav are published"<<std::endl;
            return false;
        }
    }

    // Callback for Total Station Prism
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if(this->got_transforms == false){this->get_transforms();}

        // Store latest pose
        geometry_msgs::msg::PoseWithCovarianceStamped pose_map_msg;
        tf2::doTransform(*msg, pose_map_msg, this->eigen_transform_prism_baselink);
        this->ts_point = pose_map_msg.pose.pose.position;
        this->last_ts_msg_time = this->get_clock()->now();

        // Flags to check before publishing
        if(this->got_transforms == false){return;}
        if(this->got_imu == false)
        {
            std::cout<<"No IMU data found, please check if IMU is publishing"<<std::endl;
            return;
        }
        if(this->calibration_complete == false)
        {
            if(printed_calibration_not_complete)
            {
                std::cout<<"Calibration not complete, please calibrate IMU"<<std::endl;
                printed_calibration_not_complete = true;
            }
            return;
        }

        // Print once if all flags are true
        if(printed_all_working == false)
        {
            printed_all_working = true;
            RCLCPP_INFO(this->get_logger(), "----------------------All Transforms found, publishing---------------------");
        }

        // Add yaw offset to imu orientation
        sensor_msgs::msg::Imu imu_transformed;
        tf2::doTransform(this->imu_orientation, imu_transformed.orientation, this->eigen_transform_imu_baselink);
        tf2::Quaternion q;
        tf2::fromMsg(imu_transformed.orientation, q);
        q.setRPY(0, 0, tf2::getYaw(q) + this->yaw_offset);
        imu_transformed.orientation = tf2::toMsg(q);

        // Publish pose message of TS with added IMU orientation to get a global pose source
        pose_map_msg.header.frame_id = "map";
        pose_map_msg.header.stamp = msg->header.stamp;
        pose_map_msg.pose.pose.orientation.x = imu_transformed.orientation.x;
        pose_map_msg.pose.pose.orientation.y = imu_transformed.orientation.y;
        pose_map_msg.pose.pose.orientation.z = imu_transformed.orientation.z;
        pose_map_msg.pose.pose.orientation.w = imu_transformed.orientation.w;
        pose_pub_->publish(pose_map_msg);
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RemapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
