#ifndef GOAL_HANDLER_H
#define GOAL_HANDLER_H

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/srv/berm_service.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


class GoalHandler: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        std::vector<geometry_msgs::msg::PointStamped> user_requested_berm_points_;
        std::vector<geometry_msgs::msg::PointStamped> processed_berm_points_;
        const double ANGLE_LIMIT = 50.0;
        const double INTERPOLATION_DIST = 0.4;

        // Time

        // Subscribers

        // Servers
        rclcpp::Service<lx_msgs::srv::BermService>::SharedPtr user_berm_request_server_;
        std::thread feasibility_check_thread_;

        // Clients

        // Publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr processed_berm_viz_publisher_;
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Callback function for user berm request
        * */
        void userBermRequestCB(const std::shared_ptr<lx_msgs::srv::BermService::Request> ,
                                      const std::shared_ptr<lx_msgs::srv::BermService::Response>);

        double calculateAngle(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2);

        bool areAnglesWithinRange(const std::vector<geometry_msgs::msg::PointStamped>& points, double fixedAngle);

        double distance(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2);

        std::vector<geometry_msgs::msg::PointStamped> getPointsAtFixedDistance(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2, double d);

        bool isBetween(double val, double a, double b);

        geometry_msgs::msg::PointStamped findIntersectionPoints(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2, const geometry_msgs::msg::PointStamped& p3, double d);

        /*
        * Check if berm is feasible
        * */
        void checkBermFeasibility();

        /*
        * Visualize feasible berm
        * */
        void visualizeFeasibleBerm();

        /*
        * Clean up the rviz markers
        * */
        void vizCleanup();

        // --------------------------------------

    public:
        // Functions
        /*
        * Constructor
        * */
        GoalHandler();

        /*
        * Destructor
        * */
        ~GoalHandler(){}
};

#endif