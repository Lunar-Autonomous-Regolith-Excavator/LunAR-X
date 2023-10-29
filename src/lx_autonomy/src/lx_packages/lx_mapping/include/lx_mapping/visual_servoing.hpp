#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "lx_msgs/srv/switch.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include<vector>

class VisualServoing : public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        bool debug_mode_;
        double tool_height_wrt_base_link_;
        std::thread pointcloud_thread_;
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tool_height_subscriber_;
        // Publishers
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr visual_servo_publisher_;
        // Servers
        rclcpp::Service<lx_msgs::srv::Switch>::SharedPtr visual_servo_switch_server_;
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        *
        * */
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr )

        /*
        *
        * */
        void getVisualServoError(const sensor_msgs::msg::PointCloud2::SharedPtr );

        /*
        *
        * */
        void startStopVSCallback(const std::shared_ptr<lx_msgs::srv::Switch::Request> ,
                                        std::shared_ptr<lx_msgs::srv::Switch::Response> );

        /*
        *
        * */
        void toolHeightCallback(const std_msgs::msg::Float64::SharedPtr )

    public:
        // Functions
        /*
        * Constructor
        * */
        VisualServoing();

        /*
        * Destructor
        * */
        ~VisualServoing(){};
};  

#endif
