#ifndef BERM_EVAL_H
#define BERM_EVAL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "lx_msgs/srv/switch.hpp"
#include "std_msgs/msg/float64.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>


class BermEvaluation : public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        // subscribe occupancy grid
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
        // Publishers
        // --------------------------------------
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr berm_marker_1_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr berm_marker_2_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr berm_evaluation_array_publisher_;

        std::thread berm_evaluation_thread_;

        // markers for berm
        visualization_msgs::msg::Marker berm_marker_1_;
        visualization_msgs::msg::Marker berm_marker_2_;

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        *
        * */
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr);

        /*
        *
        * */

       visualization_msgs::msg::Marker createPillarMarker(float, float, float, int);


        void bermEval(const nav_msgs::msg::OccupancyGrid::SharedPtr);


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