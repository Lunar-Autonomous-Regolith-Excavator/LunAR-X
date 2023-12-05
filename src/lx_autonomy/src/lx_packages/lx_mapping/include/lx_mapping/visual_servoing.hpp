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
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <geometry_msgs/msg/point.hpp>
#include "rclcpp/logger.hpp"
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lx_msgs/msg/berm_section.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "lx_library/lx_utils.hpp"

using namespace std;

class VisualServoing : public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        bool debug_mode_ = true;
        bool transform_mode_; // if true, then projects the berm points to the current_berm_segment
        const bool USE_MEDIAN_SEGMENTATION = true; // if true, then uses median segmentation
        bool visual_servo_fail_ = false;
        double tool_height_wrt_base_link_, tool_distance_wrt_base_link_;
        const double PCL_X_MIN_M = 0.5, PCL_X_MAX_M = 1.5; // region of interest in x direction
        const double PCL_Y_MIN_M = -0.5, PCL_Y_MAX_M = 1.0; // region of interest in y direction
        const double PCL_Z_MIN_M = -0.5, PCL_Z_MAX_M = 0.5; // region of interest in z direction
        const int NUM_BINS = 100; // number of bins in each dim the ROI
        const double MIN_PLANE_ANGLE_DEG = 10.0; // minimum angle of the plane wrt the ground plane
        const double PEAK_LINE_DISTANCE_M = 0.06; // min dist between ground plane and peak line points
        // const double DRUM_X_BASELINK_M = 0.9; // higher value -> rover stops more towards the berm
        const double DRUM_Y_BASELINK_M = 0.0; // y coordinate of the drum wrt base_link
        const double DRUM_Z_BASELINK_M = -0.28; // more negative-> higher drum
        bool node_state_ = false; // state of the node
        lx_msgs::msg::BermSection current_berm_segment, prev_berm_segment;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        // Exp filters for error values
        ExpFilter exp_filter_x_, exp_filter_y_, exp_filter_z_;
        
        std::thread pointcloud_thread_;
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tool_height_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tool_distance_subscriber_;
        // Publishers
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr peakpoints_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr groundplane_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bermplane_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr peakline_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr targetpoint_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr projected_point_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr transformed_berm_points_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr visual_servo_error_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr groundplane_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bermplane_publisher_;

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
        void publishVector(std::vector<double> , std::string);

        /*
        *
        * */
        vector<double> calculateNormalVector(pcl::ModelCoefficients::Ptr);

        /*
        *
        * */
        vector<double> binPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr , pcl::PointIndices::Ptr , vector<double> );

        /*
        *
        * */
        pcl::PointIndices::Ptr fitBestPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr , 
                                                            int ,double ,int , pcl::PointIndices::Ptr , 
                                                            pcl::ModelCoefficients::Ptr );

        /*
        *
        * */

        double getMedianElevation(pcl::PointCloud<pcl::PointXYZ>::Ptr);
        /*
        *
        */
        void getGroundIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr , 
                                                            double , pcl::PointIndices::Ptr);
        void getBermIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr , 
                                                            double , pcl::PointIndices::Ptr);
        
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

        void toolDistanceCallback(const std_msgs::msg::Float64::SharedPtr );

        vector<geometry_msgs::msg::PoseStamped> getTransformedBermSegments();

        double getTargetZ(pcl::PointCloud<pcl::PointXYZ>::Ptr);


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
