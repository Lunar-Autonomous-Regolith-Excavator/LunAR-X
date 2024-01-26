#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

#include "rclcpp/rclcpp.hpp"
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

// Convert x, y coordinates to index
#define GETMAXINDEX(x, y, width) (y * width + x)

struct BermSection {
    geometry_msgs::msg::Point center;
    double angle;

    BermSection(geometry_msgs::msg::Point center, double angle){
        this->center = center;
        this->angle = angle;
    }
};

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

class TaskPlanner: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        rclcpp::Service <lx_msgs::srv::Plan>::SharedPtr plan_service_server_;

        // Task planner variables
        std::vector<BermSection> berm_sequence_;
        std::vector<int> berm_section_iterations_;
        std::vector<double> berm_section_heights_;

        double section_length_;
        double desired_berm_height_;

        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Task planner service request callback
        * */
        void taskPlannerCallback(const std::shared_ptr<lx_msgs::srv::Plan::Request> ,
                                      std::shared_ptr<lx_msgs::srv::Plan::Response>);

        /*
        * Find berm section sequence to visit 
        * */
        bool findBermSequence(const std::vector<geometry_msgs::msg::Point>& );

        /*
        * Find excavation pose for the given berm section
        * */
        geometry_msgs::msg::Pose findExcavationPose(const BermSection& );
        
        /*
        * Find rover position to dump material for a given berm section
        * */
        geometry_msgs::msg::Pose findDumpPose(const BermSection&, const geometry_msgs::msg::Pose& );

        /*
        * Function to calculate number of depositions per segment using desired berm height
        * */
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

        /*
        * Function to get rover footprint
        * */
        std::vector<geometry_msgs::msg::Point> getRoverFootprint(const geometry_msgs::msg::Pose& );

        /*
        * Function to get bounds of rover footprint
        * */
        Bounds getBounds(const std::vector<geometry_msgs::msg::Point>& );
};

#endif