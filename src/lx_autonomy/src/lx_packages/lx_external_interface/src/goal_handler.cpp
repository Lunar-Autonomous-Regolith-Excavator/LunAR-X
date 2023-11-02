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
    processed_berm_points_.clear();
    
    RCLCPP_INFO(this->get_logger(), "Received berm request");

    // Store requested berm points 
    for(auto &point : req->berm.berm_configuration){
        user_requested_berm_points_.push_back(point);
        // Debug print points
        RCLCPP_INFO(this->get_logger(), "Point: %.2f, %.2f", point.point.x, point.point.y);
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

    // Calculate the equation of the line in the form Ax + By + C = 0
    double A = p2.y - p1.y;
    double B = p1.x - p2.x;
    double C = p2.x * p1.y - p1.x * p2.y;

    // Calculate the coefficients for the quadratic equation for y
    double ay = B * B + A * A;
    double by = 2 * B * C + 2 * A * B * p3.x - 2 * A * A * p3.y;
    double cy = C * C + 2 * A * C * p3.x + A * A * p3.x * p3.x - A * A * d * d + A * A * p3.y * p3.y;

    double ax = A * A + B * B;
    double bx = 2 * A * C + 2 * A * B * p3.y - 2 * B * B * p3.x;
    double cx = C * C + 2 * B * C * p3.y + B * B * p3.y * p3.y - B * B * d * d + B * B * p3.x * p3.x;

    // Calculate the discriminant
    double discriminanty = by * by - 4 * ay * cy;
    double discriminantx = bx * bx - 4 * ax * cx;
    double a, b, c, discriminant, x1, x2, y1, y2;
    if ((discriminantx == 0 && bx == 0) || discriminanty > 0) {
        a = ay, b = by, c = cy, discriminant = discriminanty;
        
        // Calculate the y-coordinates of the intersection points
        y1 = (-b + sqrt(discriminant)) / (2 * a);
        y2 = (-b - sqrt(discriminant)) / (2 * a);

        // Calculate the x-coordinates of the intersection points
        x1 = (-C - B * y1) / A;
        x2 = (-C - B * y2) / A;
    }

    if ((discriminanty == 0 && by == 0) || discriminantx > 0) {
        a = ax, b = bx, c = cx, discriminant = discriminantx;

        // Calculate the x-coordinates of the intersection points
        x1 = (-b + sqrt(discriminant)) / (2 * a);
        x2 = (-b - sqrt(discriminant)) / (2 * a);

        // Calculate the y-coordinates of the intersection points
        y1 = (-C - A * x1) / B;
        y2 = (-C - A * x2) / B;
    } 
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
            // 0.4 is the interpolation distance
            std::vector<geometry_msgs::msg::PointStamped> points = getPointsAtFixedDistance(first_point, user_requested_berm_points_[i+1], 0.4);
            processed_berm_points_.insert(processed_berm_points_.end(),points.begin(),points.end());
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

    // For debugging, print processed berm points
    for(auto &point : processed_berm_points_){
        RCLCPP_INFO(this->get_logger(), "Point: %.2f, %.2f", point.point.x, point.point.y);
    }
}

void GoalHandler::visualizeFeasibleBerm(){
    // TODO
}
