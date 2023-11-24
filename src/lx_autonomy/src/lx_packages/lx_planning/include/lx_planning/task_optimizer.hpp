#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/msg/planned_task.hpp"
#include "lx_msgs/srv/plan.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "lx_msgs/msg/berm_section.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/utils.h>

#include <vector>
#include <cmath>
#include <limits>

#define GETMAPINDEX(x, y, width) (y * width + x)

struct Bounds {
    double x_min;
    double x_max;
    double y_min;
    double y_max;

    Bounds() {
        x_min = std::numeric_limits<double>::max();
        x_max = std::numeric_limits<double>::min();
        y_min = std::numeric_limits<double>::max();
        y_max = std::numeric_limits<double>::min();
    }
};

struct Pose2D {
    double x;
    double y;
    double theta;

    Pose2D(double x, double y, double theta) {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }
};

class TaskPlanner: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        using TaskPlannerAction = lx_msgs::action::TaskPlanner;
        using GoalHandleTaskPlanner = rclcpp_action::ServerGoalHandle<TaskPlannerAction>;

        // Task planner variables
        std::vector<Pose2D> berm_inputs_;
        std::vector<Pose2D> excavation_poses_;
        std::vector<int> berm_section_iterations_;
        std::vector<double> berm_section_heights_;
        double section_length_;
        double desired_berm_height_;
        nav_msgs::msg::OccupancyGrid map_;

        // Servers
        rclcpp_action::Server<TaskPlannerAction>::SharedPtr taskplanner_action_server_;

        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Put next function here
        * */
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& , std::shared_ptr<const TaskPlannerAction::Goal> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action cancel request
        * */
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTaskPlanner> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action accepted
        * */
        void handle_accepted(const std::shared_ptr<GoalHandleTaskPlanner> );

        void findBermSequence(const std::shared_ptr<GoalHandleTaskPlanner> );
        
        std::vector<Pose2D> TaskPlanner::getDumpPoses(const BermSection& );

        int numOfDumps(const int );

    public:

        // Berm parameters
        static constexpr double INIT_BERM_HEIGHT = 0.09;            // m
        static constexpr double ANGLE_OF_REPOSE = 30;               // degrees

        // Rover parameters
        static constexpr double ROVER_WIDTH = 0.7;                  // m (actual width is 0.67 m)
        static constexpr double ROVER_LENGTH = 1.0;                 // m (actual length is 0.988 m)
        static constexpr double MAX_TOOL_DISTANCE_FROM_BASE = 1.0;  // m (conservative estimate for collision)
        static constexpr double TOOL_DISTANCE_TO_DUMP = 0.85;        // m

        // Functions
        /*
        * Constructor
        * */
        TaskPlanner();

        /*
        * Destructor
        * */
        ~TaskPlanner(){};

        // Function to get rover footprint
        std::vector<geometry_msgs::msg::Point> getRoverFootprint(const geometry_msgs::msg::Pose& );

        // Function to get bounds of rover footprint
        Bounds getBounds(const std::vector<geometry_msgs::msg::Point>& );
};

#endif