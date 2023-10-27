#ifndef PCL_RELAY_H
#define PCL_RELAY_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class PCLRelay: public rclcpp::Node
{
    private:
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr relay_publisher_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr aligned_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_raw_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aligned_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_raw_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_publisher_;


        void setupCommunications();

        void pclCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr );

        void alignedCallBack(const sensor_msgs::msg::Image::SharedPtr );
        void colorImageRawCallBack(const sensor_msgs::msg::Image::SharedPtr );
        void camInfoCallBack(const sensor_msgs::msg::CameraInfo::SharedPtr );


    public:
        PCLRelay();

        ~PCLRelay(){}
};

#endif