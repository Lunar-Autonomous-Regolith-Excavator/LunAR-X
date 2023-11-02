#ifndef GOAL_HANDLER_H
#define GOAL_HANDLER_H

#include "rclcpp/rclcpp.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/srv/berm_service.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <cmath>


class GoalHandler: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        std::vector<geometry_msgs::msg::PointStamped> user_requested_berm_points_;

        std::vector<geometry_msgs::msg::PointStamped> processed_berm_points_;

        // Time

        // Subscribers

        // Servers
        rclcpp::Service<lx_msgs::srv::BermService>::SharedPtr user_berm_request_server_;
        std::thread feasibility_check_thread_;

        // Clients

        // Publishers
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

        /*
        * Check if berm is feasible
        * */
        void checkBermFeasibility();

        /*
        * Visualize feasible berm
        * */
        void visualizeFeasibleBerm();

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