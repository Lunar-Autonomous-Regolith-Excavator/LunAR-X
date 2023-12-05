#ifndef PC_HANDLER_H
#define PC_HANDLER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "lx_msgs/srv/switch.hpp"
#include "std_msgs/msg/float64.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
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
#include "lx_msgs/srv/pcl_ground_height.hpp"
#include "lx_library/lx_utils.hpp"

class PointCloudHandler : public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        const double MAP_DIMENSION = 8.0;
        const double MAP_RESOLUTION = 0.05;
        const bool debug_mode_ = false;
        double tool_height_wrt_base_link_;
        bool need_ground_height_ = false;
        const double CROP_DISTANCE_FROM_TOP_M = 0.24; //more value, more crop from top
        geometry_msgs::msg::TransformStamped cam2map_transform;
        // Transforms
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        // Subscribers
        std::thread pointcloud_thread_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tool_height_subscriber_;
        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_pointcloud_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ground_height_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pointcloud_publisher_;
        // Service
        rclcpp::Service<lx_msgs::srv::PclGroundHeight>::SharedPtr pcl_ground_height_service_;
        ExpFilter exp_height_filter_;


        //
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        *
        * */
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr );

        /*
        *
        * */
        void toolHeightCallback(const std_msgs::msg::Float64::SharedPtr );

        /*
        *
        * */
        void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr );

        void pclGroundHeightCallback(const std::shared_ptr<lx_msgs::srv::PclGroundHeight::Request> ,
                                    const std::shared_ptr<lx_msgs::srv::PclGroundHeight::Response> );


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