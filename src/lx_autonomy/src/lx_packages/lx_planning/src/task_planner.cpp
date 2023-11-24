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
 * */

#include "lx_planning/task_planner.hpp"

double distanceBetweenPoses(const geometry_msgs::msg::Pose& pose_1, const geometry_msgs::msg::Pose& pose_2) {
    double x_diff = pose_1.position.x - pose_2.position.x;
    double y_diff = pose_1.position.y - pose_2.position.y;
    return std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));
}

std::vector<geometry_msgs::msg::Point> TaskPlanner::getRoverFootprint(const geometry_msgs::msg::Pose& pose) {
    
    // Generate footprint of the rover
    std::vector<geometry_msgs::msg::Point> footprint;
    geometry_msgs::msg::Point point;
    point.x = this->MAX_TOOL_DISTANCE_FROM_BASE;
    point.y = this->ROVER_WIDTH / 2;
    footprint.push_back(point);
    point.x = this->MAX_TOOL_DISTANCE_FROM_BASE;
    point.y = -this->ROVER_WIDTH / 2;
    footprint.push_back(point);
    point.x = -this->ROVER_LENGTH / 2;
    point.y = -this->ROVER_WIDTH / 2;
    footprint.push_back(point);
    point.x = -this->ROVER_LENGTH / 2;
    point.y = this->ROVER_WIDTH / 2;

    // Calculate yaw of the rover
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double yaw = tf2::getYaw(q);

    // Rotate and translate footprint
    for (int i = 0; i < static_cast<int>(footprint.size()); i++) {
        double x = footprint[i].x;
        double y = footprint[i].y;
        footprint[i].x = x * cos(yaw) - y * sin(yaw) + pose.position.x;
        footprint[i].y = x * sin(yaw) + y * cos(yaw) + pose.position.y;
    }

    return footprint;
}

Bounds TaskPlanner::getBounds(const std::vector<geometry_msgs::msg::Point>& points) {
    Bounds bounds;

    for (int i = 0; i < static_cast<int>(points.size()); i++) {
        bounds.x_min = std::min(bounds.x_min, points[i].x);
        bounds.x_max = std::max(bounds.x_max, points[i].x);
        bounds.y_min = std::min(bounds.y_min, points[i].y);
        bounds.y_max = std::max(bounds.y_max, points[i].y);
    }

    return bounds;
}

TaskPlanner::TaskPlanner(): Node("task_planner_node"){
    
    // Set up subscriptions, publishers, servers & clients
    setupCommunications();

    // initializeMap();

    RCLCPP_INFO(this->get_logger(), "Task Planner initialized");
}

void TaskPlanner::setupCommunications(){
    // Subscribers

    // Clients

    // Servers

    // Service servers
    this->plan_service_server_ = this->create_service<lx_msgs::srv::Plan>("plan_operation",
                                        std::bind(&TaskPlanner::taskPlannerCallback, this, std::placeholders::_1, std::placeholders::_2));

    
}

void TaskPlanner::taskPlannerCallback(const std::shared_ptr<lx_msgs::srv::Plan::Request> req, std::shared_ptr<lx_msgs::srv::Plan::Response> res) {
    
    RCLCPP_INFO(this->get_logger(), "Received task planning request");

    // Get points of the berm
    std::vector<geometry_msgs::msg::Point> berm_points = req->berm_input;
    desired_berm_height_ = req->berm_height;
    section_length_ = req->section_length;

    // Find berm sequence if not already found
    if (req->new_plan || this->berm_sequence_.size() == 0) {
        if (!findBermSequence(berm_points)) {
            return;
        }
    }

    // Loop through the berm sections
    for (int i = 0; i < static_cast<int>(this->berm_sequence_.size()); i++) {
        // Get the berm section
        BermSection berm_section = this->berm_sequence_[i];

        // Find the excavation pose
        geometry_msgs::msg::Pose excavation_pose = findExcavationPose(berm_section);

        // Find the dump pose
        geometry_msgs::msg::Pose dump_pose = findDumpPose(berm_section, excavation_pose);

        // Make excavation orientation same as dump orientation
        excavation_pose.orientation = dump_pose.orientation;
        
        // Update the berm section iterations if not a new plan
        if (!req->new_plan) {
            berm_section_iterations_[i] = numOfDumps(i);
        }
        // Find the number of iterations and 
        int num_iterations = berm_section_iterations_[i];

        for (int j = 0; j < num_iterations; j++){
            // Randomize excavation pose
            double min_offset = 0.1;
            double max_offset = 0.4;
            double random_offset = (double)rand() / (double)RAND_MAX; // 0 to 1
            random_offset = random_offset * (max_offset - min_offset) + min_offset; // 0.1 to 0.4
            if (rand() % 2 == 0) random_offset = -random_offset; // change sign randomly
            geometry_msgs::msg::Pose rand_excavation_pose = excavation_pose;
            rand_excavation_pose.position.y += random_offset * cos(berm_section.angle + M_PI / 2);
            rand_excavation_pose.position.y = std::max(1.5, std::min(4.5, rand_excavation_pose.position.y));

            // Add navigation task for excavation pose to the plan
            lx_msgs::msg::PlannedTask navigation_task;
            navigation_task.task_type = int(TaskTypeEnum::AUTONAV);
            navigation_task.pose = rand_excavation_pose;
            // Don't add navigation task for first excavation pose
            if (!(i == 0 && j == 0)) res->plan.push_back(navigation_task);

            // Add excavation task to the plan
            lx_msgs::msg::PlannedTask excavation_task;
            excavation_task.task_type = int(TaskTypeEnum::AUTODIG);
            excavation_task.pose = rand_excavation_pose;
            res->plan.push_back(excavation_task);

            // Add navigation task for dump pose to the plan
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

    // Add navigation task to the end pose
    geometry_msgs::msg::Pose end_pose;
    end_pose.position.x = 2;
    end_pose.position.y = 3.5;
    lx_msgs::msg::PlannedTask navigation_task;
    navigation_task.task_type = int(TaskTypeEnum::AUTONAV);
    navigation_task.pose = end_pose;
    res->plan.push_back(navigation_task);
}

bool TaskPlanner::findBermSequence(const std::vector<geometry_msgs::msg::Point> &berm_points) {
    // Clear class variables
    this->berm_sequence_.clear();
    berm_section_iterations_.clear();
    
    try {
        // Loop through the points
        for (int i = 0; i < static_cast<int>(berm_points.size()) - 1; i++){
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
            this->berm_sequence_.push_back(berm_section);
        }

        // Calculate number of iterations for each berm section
        int num_iterations = static_cast<int>(std::round(std::pow(desired_berm_height_ / INIT_BERM_HEIGHT, 2)));
        berm_section_iterations_.resize(this->berm_sequence_.size(), num_iterations);
        // Set berm heights to zero
        berm_section_heights_.resize(this->berm_sequence_.size(), 0.0);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in finding berm sequence: %s", e.what());
        return false;
    }

    return true;
}

geometry_msgs::msg::Pose TaskPlanner::findExcavationPose(const BermSection &berm_section) {
    geometry_msgs::msg::Pose excavation_pose;
    excavation_pose.position.x = berm_section.center.x;
    excavation_pose.position.y = berm_section.center.y;
    double excavation_angle = berm_section.angle + M_PI / 2;    // Doesn't matter + or - PI/2

    // Find footprint of the rover
    std::vector<geometry_msgs::msg::Point> footprint = getRoverFootprint(excavation_pose);

    // Find bounds of the footprint
    Bounds bounds = getBounds(footprint);

    // Set excavation pose
    excavation_pose.position.x = (excavation_pose.position.x - bounds.x_min) + 0.9;
    excavation_pose.position.y = tan(excavation_angle) * (1.25 - bounds.x_min) + excavation_pose.position.y;
    // Clip y position to be within bounds
    excavation_pose.position.y = std::max(1.5, std::min(4.5, excavation_pose.position.y));
    // Set orientation
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, excavation_angle);
    excavation_pose.orientation = tf2::toMsg(q);

    return excavation_pose;
}

geometry_msgs::msg::Pose TaskPlanner::findDumpPose(const BermSection &berm_section, const geometry_msgs::msg::Pose &start_pose) {
    geometry_msgs::msg::Pose dump_pose_1, dump_pose_2;
    tf2::Quaternion q;

    // Calculate dump poses on either side of the berm section with TOOL_DISTANCE_TO_DUMP from the berm section center
    double angle_1 = berm_section.angle + M_PI / 2;
    dump_pose_1.position.x = berm_section.center.x + this->TOOL_DISTANCE_TO_DUMP * cos(angle_1);
    dump_pose_1.position.y = berm_section.center.y + this->TOOL_DISTANCE_TO_DUMP * sin(angle_1);
    q.setRPY(0.0, 0.0, angle_1 - M_PI);
    dump_pose_1.orientation = tf2::toMsg(q);

    double angle_2 = berm_section.angle - M_PI / 2;
    dump_pose_2.position.x = berm_section.center.x + this->TOOL_DISTANCE_TO_DUMP * cos(angle_2);
    dump_pose_2.position.y = berm_section.center.y + this->TOOL_DISTANCE_TO_DUMP * sin(angle_2);
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