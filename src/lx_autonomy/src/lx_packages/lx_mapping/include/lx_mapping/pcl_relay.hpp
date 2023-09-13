#ifndef PCL_RELAY_H
#define PCL_RELAY_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PCLRelay: public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr relay_publisher_;

        void setupCommunications();

        void pclCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr );

    public:
        PCLRelay();

        ~PCLRelay(){}
};

#endif