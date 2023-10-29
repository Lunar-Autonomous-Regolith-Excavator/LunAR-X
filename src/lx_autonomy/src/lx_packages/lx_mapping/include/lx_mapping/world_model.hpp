#ifndef WORLD_MODEL_H
#define WORLD_MODEL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "lx_msgs/srv/switch.hpp"
#include <pcl/point_cloud.h>


class WorldModel : public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        const double MAP_DIMENSION = 8.0;
        const double MAP_RESOLUTION = 0.05;
        bool debug_mode_;
        nav_msgs::msg::OccupancyGrid global_map_, 
                                     elevation_costmap_, 
                                     slope_costmap_, 
                                     berm_costmap_, 
                                     zone_costmap_, 
                                     world_model_;
        // Subscribers
        std::thread fuse_map_thread_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_pcl_subscriber_;
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_publisher_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr world_model_publisher_;
        // Servers
        rclcpp::Service<lx_msgs::srv::Switch>::SharedPtr map_switch_server_;
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        *
        * */
        void configureMaps();

        /*
        *
        * */
        void mapSwitchCallback(const std::shared_ptr<lx_msgs::srv::Switch::Request> ,
                                std::shared_ptr<lx_msgs::srv::Switch::Response> );

        /*
        *
        * */
        void transformedPCLCallback(const sensor_msgs::msg::PointCloud2::SharedPtr );

        /*
        *
        * */
        void fuseMap(const sensor_msgs::msg::PointCloud2::SharedPtr );

        /*
        *
        * */
        void buildWorldModel();

        /*
        *
        * */
        // void buildSpecialZones(const nav_msgs::msg::Odometry::SharedPtr msg);

    public:
        // Functions
        /*
        * Constructor
        * */
        WorldModel();

        /*
        * Destructor
        * */
        ~WorldModel(){};
};  

#endif