/* Author: Hariharan Ravichandran
 * Subscribers:
 *    - /topic: description
 * Publishers:
 *    - /topic: description
 * Services:
 *    - /name (type): description
 *
 * - Summary
 * 
 * TODO
 * - One shot planning:
 *      Find the sequence of berm to be built
 * - Dynamic planning:
 *      Get berm height in each berm section and plan accordingly
 * */

#include "lx_planning/task_planner.hpp"

double distanceBetweenPoses(const geometry_msgs::msg::Pose& pose_1, const geometry_msgs::msg::Pose& pose_2) {
    double x_diff = pose_1.position.x - pose_2.position.x;
    double y_diff = pose_1.position.y - pose_2.position.y;
    return std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));
}

TaskPlanner::TaskPlanner(): Node("task_planner_node"){
    
    // Set up subscriptions, publishers, servers & clients
    setupCommunications();

    // initializeMap();

    RCLCPP_INFO(this->get_logger(), "Task Planner initialized");
}

void TaskPlanner::setupCommunications(){
    // Subscribers

    // Publishers
    rclcpp::QoS qos(10);  // initialize to default
    qos.transient_local();
    qos.reliable();
    qos.keep_last(1);
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);

    // Clients

    // Servers

    // Service servers
    this->plan_service_server_ = this->create_service<lx_msgs::srv::Plan>("plan_operation",
                                        std::bind(&TaskPlanner::taskPlannerCallback, this, std::placeholders::_1, std::placeholders::_2));

    
}

void TaskPlanner::taskPlannerCallback(const std::shared_ptr<lx_msgs::srv::Plan::Request> req, std::shared_ptr<lx_msgs::srv::Plan::Response> res) {
    
    RCLCPP_INFO(this->get_logger(), "Received task planning request");

    // Change frequency parameter of global costmap
    // TODO

    // Get points of the berm
    std::vector<geometry_msgs::msg::Point> berm_points = req->berm_input;
    desired_berm_height_ = req->berm_height;
    section_length_ = req->section_length;

    // Find berm sequence if not already found
    if (req->new_plan || berm_sequence_.size() == 0) {
        if (!findBermSequence(berm_points)) {
            return;
        }
    }

    // For each berm section, there will be an excavation task and a dump task

    /********** EXCAVATION TASK **********/
    // Assuming that the corner near the router is the origin
    // Temporary start pose for excavation at (3,1) with orientation of zero degrees throughout
    lx_msgs::msg::PlannedTask excavation_task;
    excavation_task.task_type = int(TaskTypeEnum::AUTODIG);

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = 1.25;
    start_pose.position.y = 4.0;
    start_pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0);
    start_pose.orientation = tf2::toMsg(q);
    excavation_task.pose = start_pose;

    // TEMPORARY
    drum_to_base_ = 1.0;
    berm_section_heights_.resize(berm_sequence_.size(), 0.0);

    // Loop through the berm sections
    for (int i = 0; i < berm_sequence_.size(); i++) {
        // Get the berm section
        BermSection berm_section = berm_sequence_[i];
        // Find the dump pose
        geometry_msgs::msg::Pose dump_pose = findDumpPose(berm_section, start_pose);
        // Find the number of iterations and update the berm section iterations
        int num_iterations = numOfDumps(i);
        berm_section_iterations_[i] = num_iterations;

        for (int j = 0; j < num_iterations; j++){
            // Add navigation task to the plan
            lx_msgs::msg::PlannedTask navigation_task;
            navigation_task.task_type = int(TaskTypeEnum::AUTONAV);
            navigation_task.pose = excavation_task.pose;
            if (j > 0) res->plan.push_back(navigation_task);

            // Add excavation task to the plan
            res->plan.push_back(excavation_task);
            excavation_task.pose.position.y -= 0.5;

            // Add navigation task to the plan
            navigation_task.pose = dump_pose;
            res->plan.push_back(navigation_task);

            // Add dump task to the plan
            lx_msgs::msg::PlannedTask dump_task;
            dump_task.task_type = int(TaskTypeEnum::AUTODUMP);
            dump_task.pose = dump_pose;
            // Add dump berm location to response
            dump_task.berm_point.x = berm_section.center.x;
            dump_task.berm_point.y = berm_section.center.y;
            dump_task.berm_point.theta = berm_section.angle;
            res->plan.push_back(dump_task);
        }
    }

    // Change back parameter of global costmap
    // TODO
}

bool TaskPlanner::findBermSequence(const std::vector<geometry_msgs::msg::Point> &berm_points) {
    // Clear class variables
    berm_sequence_.clear();
    berm_section_iterations_.clear();
    
    try {
        // Loop through the points
        for (int i = 0; i < berm_points.size() - 1; i++){
            // Berm end points
            geometry_msgs::msg::Point point_1 = berm_points[i];
            geometry_msgs::msg::Point point_2 = berm_points[i + 1];

            // Calculate center and angle of berm section
            geometry_msgs::msg::Point center;
            center.x = (point_1.x + point_2.x) / 2;
            center.y = (point_1.y + point_2.y) / 2;
            double angle = atan2(point_2.y - point_1.y, point_2.x - point_1.x);

            // Add to berm sequence
            BermSection berm_section(center, angle);
            berm_sequence_.push_back(berm_section);
        }

        // Calculate number of iterations for each berm section
        int num_iterations = static_cast<int>(std::round(std::pow(desired_berm_height_ / INIT_BERM_HEIGHT, 2)));
        berm_section_iterations_.resize(berm_sequence_.size(), num_iterations);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in finding berm sequence: %s", e.what());
        return false;
    }

    return true;
}

geometry_msgs::msg::Pose TaskPlanner::findDumpPose(const BermSection &berm_section, const geometry_msgs::msg::Pose &start_pose) {
    geometry_msgs::msg::Pose dump_pose_1, dump_pose_2;
    tf2::Quaternion q;

    // Calculate dump poses on either side of the berm section with drum_to_base_ distance from the berm section center
    double angle_1 = berm_section.angle + M_PI / 2;
    dump_pose_1.position.x = berm_section.center.x + drum_to_base_ * cos(angle_1);
    dump_pose_1.position.y = berm_section.center.y + drum_to_base_ * sin(angle_1);
    q.setRPY(0.0, 0.0, angle_1 - M_PI);
    dump_pose_1.orientation = tf2::toMsg(q);

    double angle_2 = berm_section.angle - M_PI / 2;
    dump_pose_2.position.x = berm_section.center.x + drum_to_base_ * cos(angle_2);
    dump_pose_2.position.y = berm_section.center.y + drum_to_base_ * sin(angle_2);
    q.setRPY(0.0, 0.0, angle_2 + M_PI);
    dump_pose_2.orientation = tf2::toMsg(q);

    // Find which is closer to start pose
    double distance_1 = distanceBetweenPoses(dump_pose_1, start_pose);
    double distance_2 = distanceBetweenPoses(dump_pose_2, start_pose);

    geometry_msgs::msg::Pose dump_pose;
    if (distance_1 < distance_2) {
        dump_pose = dump_pose_1;
    }
    else {
        dump_pose = dump_pose_2;
    }

    return dump_pose;
}

int TaskPlanner::numOfDumps(int berm_section_index) {
    // Estimate volume (area) of berm section
    double section_height = berm_section_heights_[berm_section_index];
    double est_section_width = 2 * section_height / tan(ANGLE_OF_REPOSE * M_PI / 180);
    double est_cross_section_area = 0.5 * est_section_width * section_length_;

    // Estimate volume (area) of desired berm section
    double desired_section_height = desired_berm_height_;
    double est_desired_section_width = 2 * desired_section_height / tan(ANGLE_OF_REPOSE * M_PI / 180);
    double est_desired_cross_section_area = 0.5 * est_desired_section_width * section_length_;

    // Estimate volume (area) of each dump
    double dump_height = TaskPlanner::INIT_BERM_HEIGHT;
    double est_dump_width = 2 * dump_height / tan(ANGLE_OF_REPOSE * M_PI / 180);
    double est_dump_cross_section_area = 0.5 * est_dump_width * section_length_;

    // Volume to be dumped
    double volume_to_be_dumped = std::max(est_desired_cross_section_area - est_cross_section_area, 0.0);

    // Number of dumps
    int num_dumps = static_cast<int>(std::round(volume_to_be_dumped / est_dump_cross_section_area));

    return num_dumps;
}

// Functions to publish map for costmap
// Required for Nav2 hybrid A* planner
void TaskPlanner::initializeMap() {    
    map_msg_.header.frame_id = "map";
    map_msg_.info.resolution = TaskPlanner::MAP_RESOLUTION;
    map_msg_.info.width = TaskPlanner::MAP_DIMENSION;
    map_msg_.info.height = TaskPlanner::MAP_DIMENSION;
    map_msg_.info.origin.position.x = TaskPlanner::MAP_ORIGIN_X;
    map_msg_.info.origin.position.y = TaskPlanner::MAP_ORIGIN_Y;
    map_msg_.info.origin.position.z = 0.0;
    map_msg_.info.origin.orientation.x = 0.0;
    map_msg_.info.origin.orientation.y = 0.0;
    map_msg_.info.origin.orientation.z = 0.0;
    map_msg_.info.origin.orientation.w = 1.0;

    // Set up map
    map_data_.resize(map_msg_.info.width * map_msg_.info.height, 0);
    // Make the borders of the map occupied
    for (int i = 0; i < map_msg_.info.width; i++) {
      map_msg_.data[GETMAXINDEX(i, 0, map_msg_.info.width)] = 100;
      map_msg_.data[GETMAXINDEX(i, map_msg_.info.height - 1, map_msg_.info.width)] = 100;
    }
    for (int i = 0; i < map_msg_.info.height; i++) {
      map_msg_.data[GETMAXINDEX(0, i, map_msg_.info.width)] = 100;
      map_msg_.data[GETMAXINDEX(map_msg_.info.width - 1, i, map_msg_.info.width)] = 100;
    }

    // Set up a timer to periodically publish map data
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&TaskPlanner::publishMap, this));
}

void TaskPlanner::publishMap() {
    map_msg_.header.stamp = this->get_clock()->now();
    map_msg_.info.map_load_time = this->get_clock()->now();
    map_msg_.data = map_data_;
    map_publisher_->publish(map_msg_);
}

void TaskPlanner::updateMap(const geometry_msgs::msg::Point &berm_point) {
    // TODO
    return;
}

void TaskPlanner::clearMap() {
    return;
}