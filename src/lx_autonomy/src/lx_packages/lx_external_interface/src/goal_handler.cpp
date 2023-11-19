/* Author: Vivek Chervi & Dhruv Tyagi
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
 * - Add Documentation
 * - Add check of already on-going operation
 * - Add Action Request
 * */

#include "lx_external_interface/goal_handler.hpp"

GoalHandler::GoalHandler(): Node("goal_handler_node"){  
    user_requested_berm_points_.clear();
    processed_berm_points_.clear();

    // Set up subscriptions & publishers
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "Goal Handler initialized");
}

void GoalHandler::setupCommunications(){
    // Subscribers

    // Publishers
    processed_berm_viz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lx_visualization/processed_berm", 5);

    // Clients
    berm_eval_points_client_ = this->create_client<lx_msgs::srv::BermService>("berm_evaluation/requested_berm_points");
    world_model_points_client_ = this->create_client<lx_msgs::srv::RequestRoverService>("world_model/requested_points");
    operation_action_client_ = rclcpp_action::create_client<Operation>(this, "operations/berm_build_action");

    // Servers
    user_berm_request_server_ = this->create_service<lx_msgs::srv::RequestRoverService>("request_rover/build_berm", 
                                    std::bind(&GoalHandler::userRequestCB, this, std::placeholders::_1, std::placeholders::_2));
}

void GoalHandler::userRequestCB(const std::shared_ptr<lx_msgs::srv::RequestRoverService::Request> req,
                                          const std::shared_ptr<lx_msgs::srv::RequestRoverService::Response> res){
    user_requested_berm_points_.clear();
    user_requested_excavation_points_.clear();
    user_requested_restricted_points_.clear();
    processed_berm_points_.clear();
    vizCleanup();
    
    RCLCPP_INFO(this->get_logger(), "Received berm request");

    // Store requested berm points 
    for(auto &point : req->berm.berm_configuration){
        user_requested_berm_points_.push_back(point);
        // Debug print points
        RCLCPP_INFO(this->get_logger(), "Point: %.2f, %.2f", point.point.x, point.point.y);
    }
    // Store requested excavation zone points
    for(auto &point : req->excavation_zone_coordinates){
        user_requested_excavation_points_.push_back(point);
    }
    // Store requested restricted zone points
    for(auto &point : req->restricted_zone_coordinates){
        user_requested_restricted_points_.push_back(point);
    }
    
    // Start thread to check for berm feasibility
    feasibility_check_thread_ = std::thread(&GoalHandler::checkBermFeasibility, this);

    // Detach thread
    feasibility_check_thread_.detach();

    // Set response success to indicate goal is received
    res->success = true;
}

// Function to calculate the angle between two points
double GoalHandler::calculateAngle(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2) {
    return atan2(p2.point.y - p1.point.y, p2.point.x - p1.point.x);
}

// Function to check if all angles are within a certain range with respect to the reference line
bool GoalHandler::areAnglesWithinRange(const std::vector<geometry_msgs::msg::PointStamped>& points, double fixedAngle) {
    if (points.size() < 2) {
        RCLCPP_INFO(this->get_logger(), "Not enough points to form a reference line.");
        return false;
    }

    // Convert fixed angle from degrees to radians for comparison
    fixedAngle = fixedAngle * M_PI / 180.0;

    // Calculate the angle of each line with respect to the reference line
    for (size_t i = 1; i < points.size()-1; ++i) {
        // Calculate the angle of the reference line
        double referenceAngle = calculateAngle(points[i-1], points[i]);

        double angle = calculateAngle(points[i], points[i+1]) - referenceAngle;

        // Normalize the angle to be within the range [-PI, PI)
        angle = fmod(angle + M_PI, 2 * M_PI) - M_PI;

        // Check if the angle is within the specified range
        if (std::abs(angle) > fixedAngle) {
            return false;
        }
    }

    return true;
}

double GoalHandler::distance(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2) {
    return sqrt((p2.point.x - p1.point.x) * (p2.point.x - p1.point.x) + (p2.point.y - p1.point.y) * (p2.point.y - p1.point.y));
}

std::vector<geometry_msgs::msg::PointStamped> GoalHandler::getPointsAtFixedDistance(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2, double d) {
    std::vector<geometry_msgs::msg::PointStamped> result;
    double dist = distance(p1, p2);
    double unit_vector_x = (p2.point.x - p1.point.x) / dist;
    double unit_vector_y = (p2.point.y - p1.point.y) / dist;

    for (int i = 1; i <= dist / d; ++i) {
        geometry_msgs::msg::PointStamped p;
        p.point.x = p1.point.x + i * d * unit_vector_x;
        p.point.y = p1.point.y + i * d * unit_vector_y;
        result.push_back(p);
    }

    return result;
}

bool GoalHandler::isBetween(double val, double a, double b) {
    return (val >= std::min(a, b)) && (val <= std::max(a, b));
}

geometry_msgs::msg::PointStamped GoalHandler::findIntersectionPoints(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2, const geometry_msgs::msg::PointStamped& p3, double d) {
    geometry_msgs::msg::PointStamped result;

    double dx = p2.point.x - p1.point.x;
    double dy = p2.point.y - p1.point.y;
    double dpx = p1.point.x - p3.point.x;
    double dpy = p1.point.y - p3.point.y;

    double A = dx * dx + dy * dy;
    double B = 2 * (dx * dpx + dy * dpy);
    double C = (dpx * dpx + dpy * dpy) - d * d;

    // Calculate the discriminant
    double discriminant {B * B - 4 * A * C};

    double t1 = (-B + sqrt(discriminant)) / (2 * A);
    double t2 = (-B - sqrt(discriminant)) / (2 * A);

    // Calculate the x-coordinates of the intersection points
    double x1 = p1.point.x + t1 * dx;
    double x2 = p1.point.x + t2 * dx;

    // Calculate the y-coordinates of the intersection points
    double y1 = p1.point.y + t1 * dy;
    double y2 = p1.point.y + t2 * dy;
 
    // Check if the intersection points lie on the line segment joining p1 and p2
    if (isBetween(x1, p1.point.x, p2.point.x) && isBetween(y1, p1.point.y, p2.point.y)) {
        result.point.x = x1;
        result.point.y = y1;
    }
    if (isBetween(x2, p1.point.x, p2.point.x) && isBetween(y2, p1.point.y, p2.point.y)) {
        result.point.x = x2;
        result.point.y = y2;
    }
    
    return result;
}


void GoalHandler::checkBermFeasibility(){

    RCLCPP_INFO(this->get_logger(), "Processing requested berm points");

    // Interpolate points 
    geometry_msgs::msg::PointStamped first_point{user_requested_berm_points_[0]};
    for (long unsigned int i = 0; i < user_requested_berm_points_.size()-1; i++) {
        processed_berm_points_.push_back(first_point);
        std::vector<geometry_msgs::msg::PointStamped> points = getPointsAtFixedDistance(first_point, user_requested_berm_points_[i+1], INTERPOLATION_DIST);
        processed_berm_points_.insert(processed_berm_points_.end(),points.begin(),points.end());
        if (i == user_requested_berm_points_.size()-2) break;
        geometry_msgs::msg::PointStamped line_end = processed_berm_points_.back();
        if (line_end.point.x == user_requested_berm_points_[i+1].point.x && line_end.point.y == user_requested_berm_points_[i+1].point.y) {
            first_point = user_requested_berm_points_[i+1];
        }
        else {
            first_point = findIntersectionPoints(user_requested_berm_points_[i+1], user_requested_berm_points_[i+2], line_end, INTERPOLATION_DIST);
        }               
    }

    // Visualize the processed berm
    visualizeFeasibleBerm();

    // Check if berm is feasible
    if(areAnglesWithinRange(processed_berm_points_, ANGLE_LIMIT)){
        // TODO: If feasible, send berm action request to operations handler
        RCLCPP_INFO(this->get_logger(), "Berm is feasible");
        // Check if an operation is already running
        if(operation_running_){
            RCLCPP_INFO(this->get_logger(), "An operation is already running, cancelling new goal");
            return;
        }
        // Send goal
        sendOperationGoal(processed_berm_points_);

        // Send points to berm evaluation and mapping
        sendMapPoints(processed_berm_points_, user_requested_excavation_points_, user_requested_restricted_points_);
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Berm is not feasible");
    }
}

void GoalHandler::sendOperationGoal(std::vector<geometry_msgs::msg::PointStamped> operation_berm_points){
    // Create goal
    auto build_berm_goal = Operation::Goal();
    build_berm_goal.requested_berm_config.berm_configuration = operation_berm_points;
    // Goal options
    auto send_goal_options = rclcpp_action::Client<Operation>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&GoalHandler::operationResponseCB, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoalHandler::operationFeedbackCB, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&GoalHandler::operationResultCB, this, std::placeholders::_1);
    // Send goal
    auto future_goal_handle = operation_action_client_->async_send_goal(build_berm_goal, send_goal_options);
}

void GoalHandler::operationResponseCB(GoalHandleOperation::SharedPtr future){
    auto goal_handle = future.get();
    if(!goal_handle){
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Goal was accepted by server");

    // Set operation running flag
    operation_running_ = true;
}

void GoalHandler::operationFeedbackCB(GoalHandleOperation::SharedPtr, const std::shared_ptr<const Operation::Feedback> feedback){
    RCLCPP_INFO(this->get_logger(), "Received progress : %d", feedback->progress);
}

void GoalHandler::operationResultCB(const GoalHandleOperation::WrappedResult &result){
    switch(result.code){
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(this->get_logger(), "Goal aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Unknown result code");
            break;
    }
    // Set operation running flag
    operation_running_ = false;
    // Clear berm points
    processed_berm_points_.clear();
    // Clear user requested points
    user_requested_berm_points_.clear();
    // Clean up rviz markers
    vizCleanup();
}

void GoalHandler::sendMapPoints(std::vector<geometry_msgs::msg::PointStamped> berm_eval_points, 
                                std::vector<geometry_msgs::msg::PointStamped> map_excavation_points,
                                std::vector<geometry_msgs::msg::PointStamped> map_restricted_points){
    // Send berm points to berm evaluation
    auto berm_eval_request_points = std::make_shared<lx_msgs::srv::BermService::Request>();
    berm_eval_request_points->berm.berm_configuration = berm_eval_points;

    // Send berm and special zone points to mapping
    auto world_model_request_points = std::make_shared<lx_msgs::srv::RequestRoverService::Request>();
    world_model_request_points->berm.berm_configuration = berm_eval_points;
    world_model_request_points->restricted_zone_coordinates = map_restricted_points;
    world_model_request_points->excavation_zone_coordinates = map_excavation_points;

    // Send request
    auto future_result_1 = berm_eval_points_client_->async_send_request(berm_eval_request_points , std::bind(&GoalHandler::bermEvalPointsCB, this, std::placeholders::_1));
    auto future_result_2 = world_model_points_client_->async_send_request(world_model_request_points , std::bind(&GoalHandler::worldModelPointsCB, this, std::placeholders::_1));
}

void GoalHandler::bermEvalPointsCB(rclcpp::Client<lx_msgs::srv::BermService>::SharedFuture future){
    auto status = future.wait_for(std::chrono::milliseconds(100));
    if(status == std::future_status::ready){ 
        if(future.get()->success){
            RCLCPP_INFO(this->get_logger(), "Berm evaluation node received points");
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Berm evaluation node didnt receive points");
        }
    } 
    else{
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void GoalHandler::worldModelPointsCB(rclcpp::Client<lx_msgs::srv::RequestRoverService>::SharedFuture future){
    auto status = future.wait_for(std::chrono::milliseconds(100));
    if(status == std::future_status::ready){ 
        if(future.get()->success){
            RCLCPP_INFO(this->get_logger(), "World model node received points");
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "World model node didnt receive points");
        }
    } 
    else{
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void GoalHandler::visualizeFeasibleBerm(){
    RCLCPP_INFO(this->get_logger(), "Visualizing processed berm");

    // Create visualization message
    auto processed_points_viz_message = visualization_msgs::msg::MarkerArray();
    
    if(processed_berm_points_.size() > 0){    
        // Create berm points marker
        visualization_msgs::msg::Marker berm_points_marker;
        berm_points_marker.header.frame_id = "map";
        berm_points_marker.header.stamp = this->now();
        berm_points_marker.ns = "processed_berm_points";
        berm_points_marker.id = 0;
        berm_points_marker.type = 7;
        berm_points_marker.action = 0;
        berm_points_marker.pose.position.x = 0;
        berm_points_marker.pose.position.y = 0;
        berm_points_marker.pose.position.z = 0;
        berm_points_marker.scale.x = 0.1;
        berm_points_marker.scale.y = 0.1;
        berm_points_marker.scale.z = 0.1;
        berm_points_marker.color.r = 0.0;
        berm_points_marker.color.g = 0.0;
        berm_points_marker.color.b = 1.0;
        berm_points_marker.color.a = 1.0;
        // Add points to marker
        for(auto &point : processed_berm_points_){
            geometry_msgs::msg::Point p;
            p.x = point.point.x;
            p.y = point.point.y;
            p.z = point.point.z;
            berm_points_marker.points.push_back(p);
        }
        // Add marker to message
        processed_points_viz_message.markers.push_back(berm_points_marker);

        // Create berm text marker
        visualization_msgs::msg::Marker berm_text_marker;
        berm_text_marker.header.frame_id = "map";
        berm_text_marker.header.stamp = this->now();
        berm_text_marker.ns = "processed_berm_text";
        berm_text_marker.id = 1;
        berm_text_marker.type = 9;
        berm_text_marker.action = 0;
        if(processed_berm_points_.size() > 2){
            // Calculate text position around middle point
            int berm_text_index = static_cast<int>(processed_berm_points_.size() / 2);
            berm_text_marker.pose.position.x = processed_berm_points_[berm_text_index].point.x;
            berm_text_marker.pose.position.y = processed_berm_points_[berm_text_index].point.y;
            berm_text_marker.pose.position.z = processed_berm_points_[berm_text_index].point.z + 0.3;
        }
        else{
            berm_text_marker.pose.position.x = processed_berm_points_[0].point.x;
            berm_text_marker.pose.position.y = processed_berm_points_[0].point.y;
            berm_text_marker.pose.position.z = processed_berm_points_[0].point.z + 0.3;
        }
        berm_text_marker.scale.z = 0.3;
        berm_text_marker.color.r = 0.0;
        berm_text_marker.color.g = 0.0;
        berm_text_marker.color.b = 1.0;
        berm_text_marker.color.a = 1.0;
        berm_text_marker.text = "Processed_Berm";
        // Add marker to message
        processed_points_viz_message.markers.push_back(berm_text_marker);


        // Create berm line marker
        if(processed_berm_points_.size() > 1){
            visualization_msgs::msg::Marker berm_line_marker;
            berm_line_marker.header.frame_id = "map";
            berm_line_marker.header.stamp = this->now();
            berm_line_marker.ns = "processed_berm_line";
            berm_line_marker.id = 2;
            berm_line_marker.type = 4;
            berm_line_marker.action = 0;
            berm_line_marker.pose.position.x = 0;
            berm_line_marker.pose.position.y = 0;
            berm_line_marker.pose.position.z = 0;
            berm_line_marker.scale.x = 0.03;
            berm_line_marker.color.r = 0.0;
            berm_line_marker.color.g = 0.0;
            berm_line_marker.color.b = 0.5;
            berm_line_marker.color.a = 1.0;
            // Add points to marker
            for(auto &point : processed_berm_points_){
                geometry_msgs::msg::Point p;
                p.x = point.point.x;
                p.y = point.point.y;
                p.z = point.point.z;
                berm_line_marker.points.push_back(p);
            }
            // Add marker to message
            processed_points_viz_message.markers.push_back(berm_line_marker);
        }}

        processed_berm_viz_publisher_->publish(processed_points_viz_message);
}

void GoalHandler::vizCleanup(){
    RCLCPP_INFO(this->get_logger(), "Cleaning Rviz markers");

    // Clean up the above markers from rviz
    auto processed_points_viz_message = visualization_msgs::msg::MarkerArray();
    visualization_msgs::msg::Marker berm_points_marker;
    berm_points_marker.header.frame_id = "map";
    berm_points_marker.header.stamp = this->now();
    berm_points_marker.ns = "processed_berm_points";
    berm_points_marker.id = 0;
    berm_points_marker.type = 7;
    berm_points_marker.action = 2;
    processed_points_viz_message.markers.push_back(berm_points_marker);

    visualization_msgs::msg::Marker berm_text_marker;
    berm_text_marker.header.frame_id = "map";
    berm_text_marker.header.stamp = this->now();
    berm_text_marker.ns = "processed_berm_text";
    berm_text_marker.id = 1;
    berm_text_marker.type = 9;
    berm_text_marker.action = 2;
    processed_points_viz_message.markers.push_back(berm_text_marker);

    visualization_msgs::msg::Marker berm_line_marker;
    berm_line_marker.header.frame_id = "map";
    berm_line_marker.header.stamp = this->now();
    berm_line_marker.ns = "processed_berm_line";
    berm_line_marker.id = 2;
    berm_line_marker.type = 4;
    berm_line_marker.action = 2;
    processed_points_viz_message.markers.push_back(berm_line_marker);

    processed_berm_viz_publisher_->publish(processed_points_viz_message);
}
