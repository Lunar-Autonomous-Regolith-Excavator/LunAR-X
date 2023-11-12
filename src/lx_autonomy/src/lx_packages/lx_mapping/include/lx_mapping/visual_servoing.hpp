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
#include <geometry_msgs/msg/point.hpp>
#include "rclcpp/logger.hpp"
#include <vector>

using namespace std;

// Exponential filter class
class ExpFilter{
    public:
        double DECAY_RATE;
        double prev_output;
        ExpFilter(double decay_rate = 0.9)
        {
            this->DECAY_RATE = decay_rate;
            this->prev_output = 0.0;
        }
        double getValue(double input)
        {
            this->prev_output = this->DECAY_RATE*this->prev_output + (1-this->DECAY_RATE)*input;
            return this->prev_output;
        }
};

class VisualServoing : public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        bool debug_mode_;
        double tool_height_wrt_base_link_;
        const double PCL_X_MIN_M = 0.5, PCL_X_MAX_M = 2.0; // region of interest in x direction
        const double PCL_Y_MIN_M = -0.5, PCL_Y_MAX_M = 1.0; // region of interest in y direction
        const int NUM_BINS = 100; // number of bins in each dim the ROI
        const double MIN_PLANE_ANGLE_DEG = 10.0; // minimum angle of the plane wrt the ground plane
        const double PEAK_LINE_DISTANCE_M = 0.05; // max dist between two points in the peak line
        const double DRUM_X_BASELINK_M = 0.89; // higher value -> rover stops more behind the berm
        const double DRUM_Y_BASELINK_M = 0.0; // y coordinate of the drum wrt base_link
        const double DRUM_Z_BASELINK_M = -0.3; // more negative-> higher drum
        bool node_state_ = false; // state of the node

        // Exp filters for error values
        ExpFilter exp_filter_x_, exp_filter_y_, exp_filter_z_;
        
        std::thread pointcloud_thread_;
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tool_height_subscriber_;
        // Publishers
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr peakpoints_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr groundplane_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bermplane_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr peakline_marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr targetpoint_marker_publisher_;
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
