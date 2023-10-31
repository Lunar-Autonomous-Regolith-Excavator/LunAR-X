#ifndef PC_HANDLER_H
#define PC_HANDLER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "lx_msgs/srv/switch.hpp"
#include "std_msgs/msg/float64.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

class PointCloudHandler : public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        const double MAP_DIMENSION = 8.0;
        const double MAP_RESOLUTION = 0.05;
        const bool debug_mode_ = false;
        double tool_height_wrt_base_link_;
        // Transforms
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        // Subscribers
        std::thread pointcloud_thread_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tool_height_subscriber_;
        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_pointcloud_publisher_;
        // Servers
        rclcpp::Service<lx_msgs::srv::Switch>::SharedPtr pointcloud_switch_server_;
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();
        
        /*
        *
        * */
        void pointCloudSwitchCallback(const std::shared_ptr<lx_msgs::srv::Switch::Request> ,
                                        std::shared_ptr<lx_msgs::srv::Switch::Response> );

        /*
        *
        * */
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr );


        void toolHeightCallback(const std_msgs::msg::Float64::SharedPtr );

        /*
        *
        * */
        void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr );

    public:
        // Functions
        /*
        * Constructor
        * */
        PointCloudHandler();

        /*
        * Destructor
        * */
        ~PointCloudHandler(){};
};

#endif