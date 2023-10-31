#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/msg/planned_task.hpp"
#include "lx_msgs/srv/plan.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <vector>
#include <cmath>

#ifndef PI
#define PI 3.14159265358979323846
#endif
#define GETMAXINDEX(x, y, width) (y * width + x)

struct BermSection {
    geometry_msgs::msg::Point center;
    double angle;

    BermSection(geometry_msgs::msg::Point center, double angle){
        this->center = center;
        this->angle = angle;
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

        // Robot variables
        double drum_to_base_;

        // Map variables
        std::vector<int8_t> map_data_;
        nav_msgs::msg::OccupancyGrid map_msg_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Subscribers
        
        // Clients
        
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Put next function here
        * */
        void taskPlannerCallback(const std::shared_ptr<lx_msgs::srv::Plan::Request> ,
                                      std::shared_ptr<lx_msgs::srv::Plan::Response>);

        bool findBermSequence(const std::vector<geometry_msgs::msg::Point>& );

        geometry_msgs::msg::Pose findDumpPose(const BermSection&, const geometry_msgs::msg::Pose& );

        int numOfDumps(const int );
        
        // Functions for map publishing
        void initializeMap();
        
        void publishMap();

        void updateMap(const geometry_msgs::msg::Point& );

        void clearMap();

    public:

        static constexpr double INIT_BERM_HEIGHT = 0.1;   // m
        static constexpr double ANGLE_OF_REPOSE = 30;      // degrees
        static constexpr int MAP_DIMENSION = 160;          // 8 m x 8 m
        static constexpr double MAP_RESOLUTION = 0.05;     // 5 cm
        static constexpr double MAP_ORIGIN_X = -13;    
        static constexpr double MAP_ORIGIN_Y = -9;

        // Functions
        /*
        * Constructor
        * */
        TaskPlanner();

        /*
        * Destructor
        * */
        ~TaskPlanner(){}
};

#endif