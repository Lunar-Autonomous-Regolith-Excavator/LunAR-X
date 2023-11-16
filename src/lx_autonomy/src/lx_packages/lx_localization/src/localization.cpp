/* Author: Vibhakar Mohta
 * Subscribers:
 *    - /topic: description
 * Publishers:
 *    - /topic: description
 * Services:
 *    - /name (type): description
 * Actions:
 *    - /name (type): description
 *
 * - Summary
 * 
 * TODO
 * - Add todos
 * */

#include "lx_localization/localization.hpp"

Localization::Localization() : Node("remap_msgs_localization"){

    // Setup communications
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "Localization node initialized");
}

void Localization::setupCommunications(){
    // Subscribers
    auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/husky_velocity_controller/odom", qos, std::bind(&Localization::odom_callback, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/total_station_prism", qos, std::bind(&Localization::pose_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu >("/vectornav/imu", qos, std::bind(&Localization::imu_callback, this, std::placeholders::_1));

    // Publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/husky_odom", qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/total_station_pose_map", qos);
    rover_command_pub_ = create_publisher<lx_msgs::msg::RoverCommand>("/rover_auto_cmd", 10);

    if(DEBUG_)
    {
        this->marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("localization/marker", 10);
    }

    // Actions
    calibrate_imu_action_server_ = rclcpp_action::create_server<CalibrateImu>(this, "lx_localization/calibrate_imu",
                                    std::bind(&Localization::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                    std::bind(&Localization::handle_cancel, this, std::placeholders::_1),
                                    std::bind(&Localization::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse Localization::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CalibrateImu::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received calibrate imu request");
    (void)uuid;
    (void)goal;

    // Accept and execute action
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Localization::handle_cancel(const std::shared_ptr<GoalHandleCalibrateImu> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel calibrate imu action");
    (void)goal_handle;

    // Accept cancel request
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Localization::handle_accepted(const std::shared_ptr<GoalHandleCalibrateImu> goal_handle){
    // Start execution of action
    std::thread{std::bind(&Localization::executeCalibrateIMU, this, std::placeholders::_1), goal_handle}.detach();
}

void Localization::executeCalibrateIMU(const std::shared_ptr<GoalHandleCalibrateImu> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Executing calibrate imu action");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CalibrateImu::Feedback>();
    auto result = std::make_shared<CalibrateImu::Result>();

    // Get transforms if we don't have them yet
    if(this->got_transforms_ == false)
    {
        this->get_transforms();
    }

    // If latest IMU msg and TS msg are more than 1 second away from current time, return error
    auto current_time = this->get_clock()->now();
    if((current_time - this->last_imu_msg_time_).seconds() > 1.0 || (current_time - this->last_ts_msg_time_).seconds() > 1.0)
    {
        RCLCPP_INFO(this->get_logger(), "No IMU or TS data found, please check if IMU and TS are publishing");
        // Set result
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    // Get latest IMU yaw in base_link frame and rover pose
    geometry_msgs::msg::Quaternion imu_orientation = this->imu_orientation_;
    tf2::doTransform(imu_orientation, imu_orientation, this->eigen_transform_imu_baselink_);
    auto yaw_initial = tf2::getYaw(imu_orientation);
    geometry_msgs::msg::Point init_ts_point = this->ts_point_;

    // Move the rover ahead for 5 seconds
    auto start_time = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Starting calibration movement");
    lx_msgs::msg::RoverCommand rover_command;
    rclcpp::Rate loop_rate(10);
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
        rover_command.mobility_twist.linear.x = 0.1;
        rover_command_pub_->publish(rover_command);
        loop_rate.sleep();
    }
    // Stop the rover
    rover_command.mobility_twist.linear.x = 0.0;
    rover_command_pub_->publish(rover_command);

    if (goal_handle->is_canceling() || !rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(), "Calibrate imu action cancelled");
        // Set result
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    // If latest IMU msg and TS msg are more than 1 second away from current time, return error
    current_time = this->get_clock()->now();
    if((current_time - this->last_imu_msg_time_).seconds() > 1.0 || (current_time - this->last_ts_msg_time_).seconds() > 1.0)
    {
        RCLCPP_INFO(this->get_logger(), "No IMU or TS data found, please check if IMU and TS are publishing");
        // Set result
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    // Record final yaw (in base_link frame) and rover pose
    geometry_msgs::msg::Quaternion new_imu_orientation = this->imu_orientation_;
    tf2::doTransform(new_imu_orientation, new_imu_orientation, this->eigen_transform_imu_baselink_);
    auto yaw_final = tf2::getYaw(new_imu_orientation);
    geometry_msgs::msg::Point final_ts_point = this->ts_point_;

    // Calculate yaw offset
    auto avg_imu_yaw = (yaw_initial + yaw_final) / 2.0;
    auto yaw_total_station = atan2(final_ts_point.y - init_ts_point.y, final_ts_point.x - init_ts_point.x);
    this->yaw_offset_ = yaw_total_station - avg_imu_yaw;

    RCLCPP_INFO(this->get_logger(), "Calibration complete, with yaw offset: %f", this->yaw_offset_);
    this->calibration_complete_ = true;

    // Set result
    result->success = true;
    goal_handle->succeed(result);
}

void Localization::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    // Update the frame_id field in the message
    msg->header.frame_id = "base_link";
    // reset covariance
    msg->twist.covariance[0] = 0.01; // vX
    msg->twist.covariance[7] = 0.05; // vy
    msg->twist.covariance[14] = 0.01; // vz
    msg->twist.covariance[21] = 0.01; // roll
    msg->twist.covariance[28] = 0.01; // pitch
    msg->twist.covariance[35] = 0.1; // yaw
    odom_pub_->publish(*msg);
}

void Localization::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
    got_imu_ = true;
    this->imu_orientation_ = msg->orientation;
    this->last_imu_msg_time_ = this->get_clock()->now();
}

bool Localization::get_transforms(){
    try{
        std::cout<<"Looking for transform"<<std::endl;
        // Get transform matrix from total_station_prism to base_link
        tf2_ros::Buffer tf_buffer(this->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);
        this->eigen_transform_prism_baselink_ = tf_buffer.lookupTransform("base_link", "total_station_prism", tf2::TimePointZero, tf2::durationFromSec(0.5)); //prism to base link
        this->eigen_transform_imu_baselink_ = tf_buffer.lookupTransform("base_link", "vectornav", tf2::TimePointZero, tf2::durationFromSec(0.5)); //primm to base link
        this->got_transforms_ = true;
        return true;
    } 
    catch (tf2::TransformException& ex) {
        std::cout<<"No transforms found, please check if base_link to total_station_prism and base_link to vectornav are published"<<std::endl;
        return false;
    }
}

// Callback for Total Station Prism
void Localization::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    if(this->got_transforms_ == false){this->get_transforms();}

    // Store latest pose
    this->ts_point_ = msg->pose.pose.position;
    this->last_ts_msg_time_ = this->get_clock()->now();

    // Flags to check before publishing
    if(this->got_transforms_ == false){return;}
    if(this->got_imu_ == false)
    {
        std::cout<<"No IMU data found, please check if IMU is publishing"<<std::endl;
        return;
    }
    if(this->calibration_complete_ == false)
    {
        if(printed_calibration_not_complete_)
        {
            std::cout<<"Calibration not complete, please calibrate IMU"<<std::endl;
            printed_calibration_not_complete_ = true;
        }
        return;
    }

    // Print once if all flags are true
    if(printed_all_working_ == false)
    {
        printed_all_working_ = true;
        RCLCPP_INFO(this->get_logger(), "----------------------All Transforms found, publishing---------------------");
    }

    // Add yaw offset to imu orientation
    sensor_msgs::msg::Imu imu_transformed;
    tf2::doTransform(this->imu_orientation_, imu_transformed.orientation, this->eigen_transform_imu_baselink_);
    double yaw, pitch, roll;
    tf2::getEulerYPR(imu_transformed.orientation, yaw, pitch, roll);
    tf2::Quaternion q_map_to_baselink;
    q_map_to_baselink.setRPY(-roll, pitch, yaw+this->yaw_offset_);

    // Get position of baselink in map frame 
    // NOTE: 
    //   - TS raw position is the position of the total_station_prism in the map frame
    //   - This needs to be transformed to the position of the base_link in the map frame for fusion with the robot_localization package
    //   - The prism to baselink translation is transformed to the map frame and added to raw TS position to get the baselink position in map frame
    Eigen::Quaternion<double> R_map_to_baselink(q_map_to_baselink.w(), q_map_to_baselink.x(), q_map_to_baselink.y(), q_map_to_baselink.z());
    Eigen::Vector3d ts_to_baselink_vec(
        eigen_transform_prism_baselink_.transform.translation.x,
        eigen_transform_prism_baselink_.transform.translation.y,
        eigen_transform_prism_baselink_.transform.translation.z
    );
    Eigen::Vector3d rotated_ts_to_baselink_vec = R_map_to_baselink * ts_to_baselink_vec;
    Eigen::Vector3d map_to_ts_vec(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    Eigen::Vector3d map_to_baselink_vec = map_to_ts_vec - rotated_ts_to_baselink_vec;

    // Publish pose of TS in map frame
    geometry_msgs::msg::PoseWithCovarianceStamped pose_map_msg;
    pose_map_msg.header.frame_id = "map";
    pose_map_msg.header.stamp = this->get_clock()->now();
    pose_map_msg.pose.pose.position.x = map_to_baselink_vec.x();
    pose_map_msg.pose.pose.position.y = map_to_baselink_vec.y();
    pose_map_msg.pose.pose.position.z = map_to_baselink_vec.z();
    pose_map_msg.pose.pose.orientation.x = q_map_to_baselink.x();
    pose_map_msg.pose.pose.orientation.y = q_map_to_baselink.y();
    pose_map_msg.pose.pose.orientation.z = q_map_to_baselink.z();
    pose_map_msg.pose.pose.orientation.w = q_map_to_baselink.w();
    pose_pub_->publish(pose_map_msg);

    if (DEBUG_)
    {
        // Publish marker at pose_map_msg.pose.pose.position
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "localization";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = pose_map_msg.pose.pose.position.x;
        marker.pose.position.y = pose_map_msg.pose.pose.position.y;
        marker.pose.position.z = pose_map_msg.pose.pose.position.z;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        this->marker_pub_->publish(marker);
    }
}