#ifndef GOAL_HANDLER_H
#define GOAL_HANDLER_H

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/srv/berm_service.hpp"
#include "lx_msgs/srv/request_rover_service.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "lx_msgs/action/operation.hpp"
#include "lx_msgs/msg/berm_config.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class GoalHandler: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        using Operation = lx_msgs::action::Operation;
        using GoalHandleOperation = rclcpp_action::ClientGoalHandle<Operation>;
        std::vector<geometry_msgs::msg::PointStamped> user_requested_berm_points_;
        std::vector<geometry_msgs::msg::PointStamped> user_requested_excavation_points_;
        std::vector<geometry_msgs::msg::PointStamped> user_requested_restricted_points_;
        std::vector<geometry_msgs::msg::PointStamped> processed_berm_points_;
        const double ANGLE_LIMIT = 50.0;
        const double INTERPOLATION_DIST = GLOBAL_BERM_LENGTH_M;
        bool operation_running_ = false;
        // Servers
        rclcpp::Service<lx_msgs::srv::RequestRoverService>::SharedPtr user_berm_request_server_;
        std::thread feasibility_check_thread_;
        // Service Clients
        rclcpp::Client<lx_msgs::srv::BermService>::SharedPtr berm_eval_points_client_;
        rclcpp::Client<lx_msgs::srv::RequestRoverService>::SharedPtr world_model_points_client_;
        // Action Clients
        rclcpp_action::Client<Operation>::SharedPtr operation_action_client_;
        // Publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr processed_berm_viz_publisher_;
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Callback function for user request
        * */
        void userRequestCB(const std::shared_ptr<lx_msgs::srv::RequestRoverService::Request> ,
                                      const std::shared_ptr<lx_msgs::srv::RequestRoverService::Response>);

        /*
        * Calculate angle between 2 points
        * */
        double calculateAngle(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2);

        /*
        * Calculate if angles are within limits
        * */
        bool areAnglesWithinRange(const std::vector<geometry_msgs::msg::PointStamped>& points, double fixedAngle);

        /*
        * Calculate distance between 2 points
        * */
        double distance(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2);

        /*
        * Get point at a fixed distance between 2 points
        * */
        std::vector<geometry_msgs::msg::PointStamped> getPointsAtFixedDistance(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2, double d);

        /*
        * Check if a value is between a range
        * */
        bool isBetween(double val, double a, double b);

        /*
        * Calculate intersection points
        * */
        geometry_msgs::msg::PointStamped findIntersectionPoints(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2, const geometry_msgs::msg::PointStamped& p3, double d);

        /*
        * Check if berm is feasible
        * */
        void checkBermFeasibility();

        /*
        * Send goal to rover operation handler
        * */
        void sendOperationGoal(std::vector<geometry_msgs::msg::PointStamped> );

        /*
        * Handle response from operation handler
        * */
        void operationResponseCB(GoalHandleOperation::SharedPtr);

        /*
        * Handle feedback from operation handler
        * */
        void operationFeedbackCB(GoalHandleOperation::SharedPtr, const std::shared_ptr<const Operation::Feedback> );

        /*
        * Handle result from operation handler
        * */
        void operationResultCB(const GoalHandleOperation::WrappedResult&);

        /*
        * Send special map points to mapping pipeline
        * */
        void sendMapPoints(std::vector<geometry_msgs::msg::PointStamped> , 
                            std::vector<geometry_msgs::msg::PointStamped> ,
                            std::vector<geometry_msgs::msg::PointStamped> );

        /*
        * Callback for berm evaluation points from user
        * */
        void bermEvalPointsCB(rclcpp::Client<lx_msgs::srv::BermService>::SharedFuture );

        /*
        * Callback for providing points to the mapping pipeline
        * */
        void worldModelPointsCB(rclcpp::Client<lx_msgs::srv::RequestRoverService>::SharedFuture );

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