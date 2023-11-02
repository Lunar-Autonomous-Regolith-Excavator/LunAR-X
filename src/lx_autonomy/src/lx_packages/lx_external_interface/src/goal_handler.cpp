/* Author: Dhruv Tyagi
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
 * - Add Visualization
 * - Add check of already on-going operation
 * - Add Feasibility Check
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

    // Clients

    // Servers
    user_berm_request_server_ = this->create_service<lx_msgs::srv::BermService>("request_rover/build_berm", 
                                    std::bind(&GoalHandler::userBermRequestCB, this, std::placeholders::_1, std::placeholders::_2));
}

void GoalHandler::userBermRequestCB(const std::shared_ptr<lx_msgs::srv::BermService::Request> req,
                                          const std::shared_ptr<lx_msgs::srv::BermService::Response> res){
    user_requested_berm_points_.clear();
    RCLCPP_INFO(this->get_logger(), "Received berm request");

    // Store requested berm points 
    for(auto &point : req->berm.berm_configuration){
        user_requested_berm_points_.push_back(point);
        // Debug print points
        RCLCPP_INFO(this->get_logger(), "Point: %.2f, %.2f", point.point.x, point.point.point.y);
    }
    
    // Start thread to check for berm feasibility
    feasibility_check_thread_ = std::thread(&GoalHandler::checkBermFeasibility, this);

    // Detach thread
    feasibility_check_thread_.detach();

    // Set response success to indicate goal is received
    res->success = true;
}

// Function to calculate the angle between two points
double calculateAngle(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2) {
    return atan2(p2.point.y - p1.point.y, p2.point.x - p1.point.x);
}

// Function to check if all angles are within a certain range with respect to the reference line
bool areAnglesWithinRange(const std::vector<geometry_msgs::msg::PointStamped>& points, double fixedAngle) {
    if (points.size() < 2) {
        RCLCPP_INFO(this->get_logger(), "Not enough points to form a reference line.");
        return false;
    }

    // Convert fixed angle from degrees to radians for comparison
    fixedAngle = fixedAngle * M_PI / 180.0;

    // Calculate the angle of the reference line
    double referenceAngle = calculateAngle(points[0], points[1]);

    // Calculate the angle of each line with respect to the reference line
    for (size_t i = 2; i < points.size(); ++i) {
        double angle = calculateAngle(points[0], points[i]) - referenceAngle;

        // Normalize the angle to be within the range [-PI, PI)
        angle = fmod(angle + M_PI, 2 * M_PI) - M_PI;

        // Check if the angle is within the specified range
        if (std::abs(angle) > fixedAngle) {
            return false;
        }
    }

    return true;
}

double distance(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2) {
    return sqrt((p2.point.x - p1.point.x) * (p2.point.x - p1.point.x) + (p2.point.y - p1.point.y) * (p2.point.y - p1.point.y));
}

std::vector<geometry_msgs::msg::PointStamped> getPointsAtFixedDistance(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2, double d) {
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

bool isBetween(double val, double a, double b) {
    return (val >= std::min(a, b)) && (val <= std::max(a, b));
}

Point findIntersectionPoints(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2, const geometry_msgs::msg::PointStamped& p3, double d) {
    Point result;

    // Calculate the equation of the line in the form Ax + By + C = 0
    double A = p2.point.y - p1.point.y;
    double B = p1.point.x - p2.point.x;
    double C = p2.point.x * p1.point.y - p1.point.x * p2.point.y;

    // Calculate the coefficients for the quadratic equation
    double a = A * A + B * B;
    double b = 2 * A * C + 2 * A * B * p3.point.y - 2 * B * B * p3.point.x;
    double c = C * C + 2 * B * C * p3.point.y + B * B * p3.point.y * p3.point.y - B * B * d * d + B * B * p3.point.x * p3.point.x;

    // Calculate the discriminant
    double discriminant = b * b - 4 * a * c;

    if (discriminant >= 0) {
        // Calculate the x-coordinates of the intersection points
        double x1 = (-b + sqrt(discriminant)) / (2 * a);
        double x2 = (-b - sqrt(discriminant)) / (2 * a);

        // Calculate the y-coordinates of the intersection points
        double y1 = (-C - A * x1) / B;
        double y2 = (-C - A * x2) / B;

        // Check if the intersection points lie on the line segment joining p1 and p2
        if (isBetween(x1, p1.point.x, p2.point.x) && isBetween(y1, p1.point.y, p2.point.y)) {
            result = {x1, y1};
        }
        if (isBetween(x2, p1.point.x, p2.point.x) && isBetween(y2, p1.point.y, p2.point.y)) {
            result = {x2, y2};
        }
    }

    return result;
}


void GoalHandler::checkBermFeasibility(){
    // TODO
    RCLCPP_INFO(this->get_logger(), "Checking berm feasibility");

    // Check if berm is feasible
    bool isFeasible = areAnglesWithinRange(user_requested_berm_points_, 50.0);

    // If feasible, interpolate points 
    if (isFeasible) {
        RCLCPP_INFO(this->get_logger(), "Berm is feasible");
        geometry_msgs::msg::PointStamped first_point{user_requested_berm_points_[0]};
        for (int i=0; i<user_requested_berm_points_.size()-1; i++) {
            processed_berm_points_.push_back(first_point);
            std::vector<geometry_msgs::msg::PointStamped> points = getPointsAtFixedDistance(first_point, user_requested_berm_points_[i+1], d);
            processed_berm_points_.insert(result.end(),points.begin(),points.end());
            if (i == user_requested_berm_points_.size()-2) break;
            geometry_msgs::msg::PointStamped line_end = points.back();
            if (line_end.point.x == user_requested_berm_points_[i+1].point.x && line_end.point.y == user_requested_berm_points_[i+1].point.y) {
                first_point = user_requested_berm_points_[i+1];
            }
            else {
                // 0.4 is the interpolation distance
                first_point = findIntersectionPoints(user_requested_berm_points_[i+1], user_requested_berm_points_[i+2], line_end, 0.4);
            }               
        }

    // If feasible, send berm action request to operations handler

    }
}

void GoalHandler::visualizeFeasibleBerm(){
    // TODO
}
