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

double addAngles(double angle1, double angle2)
{
    // sum clips to -pi to pi
    double sum = angle1 + angle2;
    if(sum > M_PI) sum -= 2.0 * M_PI;
    else if(sum < -M_PI) sum += 2.0 * M_PI;
    return sum;
}

void Localization::append_TS_IMU_Data(std::vector<std::pair<geometry_msgs::msg::Point, double>> & TS_IMU_Data, int itr)
{
    auto current_time = this->get_clock()->now();
    double time_diff_imu = (current_time - this->last_imu_msg_time_).seconds(), time_diff_ts = (current_time - this->last_ts_msg_time_).seconds();
    if(time_diff_imu > 1.5 || time_diff_ts > 1.5)
    {
        if (time_diff_imu > 1.5) RCLCPP_INFO(this->get_logger(), "No IMU data found in the last 1.5 second of the initial point in calibration, at itr: %d", itr);
        if (time_diff_ts > 1.5) RCLCPP_INFO(this->get_logger(), "No TS data found in the last 1.5 second of the initial point in calibration, found in %f seconds", time_diff_ts);
    }
    else
    {
        // Get latest IMU yaw in base_link frame and rover pose
        geometry_msgs::msg::Quaternion imu_orientation = this->imu_orientation_;
        tf2::doTransform(imu_orientation, imu_orientation, this->eigen_transform_imu_baselink_);
        auto imu_yaw = tf2::getYaw(imu_orientation);
        TS_IMU_Data.push_back(std::make_pair(this->ts_point_, imu_yaw));
    }
}

void Localization::executeCalibrateIMU(const std::shared_ptr<GoalHandleCalibrateImu> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Executing calibrate IMU action");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CalibrateImu::Feedback>();
    auto result = std::make_shared<CalibrateImu::Result>();

    // Get transforms if we don't have them yet
    if(this->got_transforms_ == false)
    {
        this->get_transforms();
    }
    
    // Check if we have transforms
    if(this->got_transforms_ == false)
    {
        RCLCPP_INFO(this->get_logger(), "No transforms found, please check if base_link to total_station_prism and base_link to vectornav are published");
        // Set result
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    std::vector<std::pair<geometry_msgs::msg::Point, double>> TS_IMU_Data;

    // Extract the movement time, with default value of 6.5 seconds
    double move_time = goal->time;
    if(goal->dont_move_rover == false) move_time = 6.5; // for manual call

    // Fill in TS_IMU_Data every 2 seconds for the next goal->time_s seconds
    // Move the rover if dont_move_rover is false
    int itr = 0;
    auto start_time = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Starting calibration");
    
    lx_msgs::msg::RoverCommand rover_command;
    rclcpp::Rate loop_rate(1);

    while(rclcpp::ok() && !goal_handle->is_canceling())
    {
        // Check if 5 seconds have passed
        auto current_time = this->get_clock()->now();
        auto time_diff = current_time - start_time;
        
        if(time_diff.seconds() >= move_time)
        {
            RCLCPP_INFO(this->get_logger(), "Calibration movement complete");
            break;
        }
        
        // Move rover if dont_move_rover is false
        if (goal->dont_move_rover == false)
        {
            // Publish rover command
            rover_command.mobility_twist.linear.x = 0.1;
            rover_command_pub_->publish(rover_command);
        }
        
        // Try to record TS and IMU data every 2 seconds
        if (itr++ % 3 == 0) {
            append_TS_IMU_Data(TS_IMU_Data, itr);
        }

        loop_rate.sleep();
    }
    
    // Stop the rover
    if (goal->dont_move_rover == false)
    {
        rover_command.mobility_twist.linear.x = 0.0;
        rover_command_pub_->publish(rover_command);
    }

    // Return if goal is cancelled or node is shutting down
    if (goal_handle->is_canceling() || !rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(), "Calibrate imu action cancelled");
        // Set result
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    // If no points could be recorded, return error
    if(TS_IMU_Data.size() == 0)
    {
        RCLCPP_INFO(this->get_logger(), "No data recorded, aborting calibration");
        // Set result
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    // Compute all yaw offsets
    std::vector<double> yaw_offsets;
    for(int i=0; i < ((int)TS_IMU_Data.size()-1); i++)
    {
        // if ith and i+1 th yaw differ by more than 10 degrees, skip this point
        if (abs(TS_IMU_Data[i].second - TS_IMU_Data[i+1].second) > 10.0 * M_PI / 180.0)
        {
            continue;
        }

        // Calculate yaw offset
        auto avg_imu_yaw = addAngles((TS_IMU_Data[i].second + TS_IMU_Data[i+1].second) / 2.0, 0);
        auto yaw_total_station = atan2(TS_IMU_Data[i+1].first.y - TS_IMU_Data[i].first.y, TS_IMU_Data[i+1].first.x - TS_IMU_Data[i].first.x);
        double yaw_offset = yaw_total_station - avg_imu_yaw;
        yaw_offset = addAngles(yaw_offset, 0);
        yaw_offsets.push_back(yaw_offset);
    }

    // If no yaw offsets could be calculated, return error
    if(yaw_offsets.size() == 0)
    {
        RCLCPP_INFO(this->get_logger(), "No yaw offsets could be calculated, aborting calibration");
        // Set result
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    // Compute average yaw offset
    double new_yaw_offset = 0;
    for(auto yaw_offset : yaw_offsets)
    {
        new_yaw_offset += yaw_offset;
    }
    new_yaw_offset = new_yaw_offset / (double) yaw_offsets.size(); // average yaw offset

    // If we are in dont_move_rover mode, and the new offset differs from the old offset by more than 15 degrees, return error
    if(goal->dont_move_rover == true && abs(new_yaw_offset - this->yaw_offset_) > 80.0 * M_PI / 180.0)
    {
        RCLCPP_INFO(this->get_logger(), "New yaw offset %f differs from old yaw offset %f by more than 80 degrees, calibration failed", new_yaw_offset, this->yaw_offset_);
        // Set result
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    // Update yaw offset
    this->yaw_offset_ = new_yaw_offset;

    // Print yaw offset
    RCLCPP_INFO(this->get_logger(), "Calibration complete, with avg yaw offset %f", this->yaw_offset_); 
    std::stringstream ss;
    ss << "Yaw offsets: ";
    for(auto yaw_offset : yaw_offsets)
    {
        ss << yaw_offset << ", ";
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());

    // Set calibration complete flag
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
    q_map_to_baselink.setRPY(-roll, pitch, addAngles(yaw, this->yaw_offset_));

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