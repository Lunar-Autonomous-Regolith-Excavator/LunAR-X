#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "lx_msgs/srv/switch.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/msg/point.hpp>
#include "rclcpp/logger.hpp"
#include<vector>

using namespace std;


class VisualServoing : public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        bool debug_mode_;
        double tool_height_wrt_base_link_;
        const double PCL_X_MIN_M = 0.5, PCL_X_MAX_M = 2.0; // region of interest in x direction
        const double PCL_Y_MIN_M = -0.5, PCL_Y_MAX_M = 1.0; // region of interest in y direction
        const int NUM_BINS = 50; // number of bins in each dim the ROI
        const double MIN_PLANE_ANGLE_DEG = 15.0; // minimum angle of the plane wrt the ground plane
        const double PEAK_LINE_DISTANCE_M = 0.05; // max dist between two points in the peak line
        const double DRUM_X_BASELINK_M = 0.5; // x coordinate of the drum wrt base_link
        const double DRUM_Y_BASELINK_M = 0.0; // y coordinate of the drum wrt base_link
        
        std::thread pointcloud_thread_;
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tool_height_subscriber_;
        // Publishers
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visual_servo_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr groundplane_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bermplane_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr peakline_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr targetpoint_marker_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr visual_servo_error_publisher_;

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
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr );

        /*
        *
        * */

        void publishVector(std::vector<double> v, std::string);
        vector<double> calculateNormalVector(pcl::ModelCoefficients::Ptr);
        vector<double> binPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointIndices::Ptr inliers, vector<double> ground_plane_equation);

        pcl::PointCloud<pcl::PointXYZ>::Ptr  processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr );

        /*
        *
        */

        void bestPlaneFit(pcl::PointCloud<pcl::PointXYZ>::Ptr , int , double , int , pcl::PointIndices::Ptr , pcl::ModelCoefficients::Ptr );
        /*
        *
        */
        // define crossProduct function
        vector<double> crossProduct(std::vector<double>, std::vector<double>);
        /*
        *
        */

        void getVisualServoError(const sensor_msgs::msg::PointCloud2::SharedPtr );

        /*
        *
        * */
        void startStopVSCallback(const std::shared_ptr<lx_msgs::srv::Switch::Request> ,
                                        std::shared_ptr<lx_msgs::srv::Switch::Response> );

        /*
        *
        * */
        void toolHeightCallback(const std_msgs::msg::Float64::SharedPtr );

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
