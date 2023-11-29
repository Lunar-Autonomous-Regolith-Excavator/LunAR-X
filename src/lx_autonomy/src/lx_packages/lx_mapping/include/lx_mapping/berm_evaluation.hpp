#ifndef BERM_EVAL_H
#define BERM_EVAL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "lx_msgs/srv/switch.hpp"
#include "std_msgs/msg/float64.hpp"
#include "lx_msgs/srv/berm_service.hpp"
#include "lx_msgs/srv/berm_progress_eval.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include "lx_library/lx_utils.hpp"


class BermEvaluation : public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        const double ELEVATION_SCALE = 400;
        const double DESIRED_BERM_HEIGHT_M = GLOBAL_BERM_HEIGHT_M;
        const double MIN_BERM_HEIGHT_M = 0.05;
        const bool DEBUG_MODE = true;
        std::vector<geometry_msgs::msg::PointStamped> requested_berm_points_;
        nav_msgs::msg::OccupancyGrid::SharedPtr map_;
        lx_msgs::msg::BermProgress berm_progress_;
        // make dictionary of double vectors, to store volume with timestamps
        std::map<double, std::vector<double>> volumetric_progress_;

        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
        // Publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr berm_evaluation_array_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr peak_points_publisher_;

        std::thread map_update_thread_;
        // Servers
        rclcpp::Service<lx_msgs::srv::BermService>::SharedPtr berm_points_server_;
        rclcpp::Service<lx_msgs::srv::BermProgressEval>::SharedPtr berm_eval_server_;

        // markers for berm
        visualization_msgs::msg::Marker berm_marker_1_;
        visualization_msgs::msg::Marker berm_marker_2_;

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        void userBermPointsCB(const std::shared_ptr<lx_msgs::srv::BermService::Request> ,
                                          const std::shared_ptr<lx_msgs::srv::BermService::Response> );

        /*
        *
        * */
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr);

        /*
        *
        * */

        void evalServiceCallback(const std::shared_ptr<lx_msgs::srv::BermProgressEval::Request> ,
                                          const std::shared_ptr<lx_msgs::srv::BermProgressEval::Response> );

        /*
        *
        * */

        // visualization_msgs::msg::Marker createPillarMarker(float, float, float, int);

        void saveUpdatedMap(const nav_msgs::msg::OccupancyGrid::SharedPtr);


    public:
        // Functions
        /*
        * Constructor
        * */
        BermEvaluation();

        /*
        * Destructor
        * */
        ~BermEvaluation(){};
};

#endif