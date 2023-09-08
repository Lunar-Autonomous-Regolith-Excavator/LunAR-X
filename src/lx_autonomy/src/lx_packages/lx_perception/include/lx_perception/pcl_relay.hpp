#ifndef PCL_RELAY_H
#define PCL_RELAY_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

class PCLRelay: public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr relay_publisher_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_publisher_;

        void setupCommunications();

        void pclCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr );
        void camCallBack(const sensor_msgs::msg::Image::SharedPtr );

    public:
        PCLRelay();

        ~PCLRelay(){}
};

#endif