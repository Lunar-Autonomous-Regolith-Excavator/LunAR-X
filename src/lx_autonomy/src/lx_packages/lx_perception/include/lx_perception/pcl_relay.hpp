#ifndef PCL_RELAY_H
#define PCL_RELAY_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "lx_msgs/msg/tool_info.hpp"

class PCLRelay: public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_publisher_;

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tool_raw_info_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tool_raw_info_publisher_;

        rclcpp::Subscription<lx_msgs::msg::ToolInfo>::SharedPtr tool_info_subscriber_;
        rclcpp::Publisher<lx_msgs::msg::ToolInfo>::SharedPtr tool_info_publisher_;

        void setupCommunications();

        void pclCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr );
        void camCallBack(const sensor_msgs::msg::Image::SharedPtr );
        void toolRawInfoCallBack(const std_msgs::msg::Float64MultiArray::SharedPtr );
        void toolInfoCallBack(const lx_msgs::msg::ToolInfo::SharedPtr );

    public:
        PCLRelay();

        ~PCLRelay(){}
};

#endif