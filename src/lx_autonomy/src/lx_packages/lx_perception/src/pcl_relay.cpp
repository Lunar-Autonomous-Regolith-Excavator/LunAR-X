#include "lx_perception/pcl_relay.hpp"

PCLRelay::PCLRelay(): Node("pcl_relay_node"){
    // Set up subscriptions & publishers
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "PCL Relay initialized");
}

void PCLRelay::setupCommunications(){
    // Set the QoS settings best effort
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // Subscribers
    pcl_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points", 1 , 
                            std::bind(&PCLRelay::pclCallBack, this, std::placeholders::_1));
    
    cam_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", qos, std::bind(&PCLRelay::camCallBack, this, std::placeholders::_1));
    
    tool_raw_info_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/tool_raw_info", qos, std::bind(&PCLRelay::toolRawInfoCallBack, this, std::placeholders::_1));
    
    tool_info_subscriber_ = this->create_subscription<lx_msgs::msg::ToolInfo>(
      "/tool_info", qos, std::bind(&PCLRelay::toolInfoCallBack, this, std::placeholders::_1));
    
    // Publishers
    pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_relay", qos);
    cam_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("cam_relay", qos);
    tool_raw_info_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("tool_raw_info_relay", qos);
    tool_info_publisher_ = this->create_publisher<lx_msgs::msg::ToolInfo>("tool_info_relay", qos);
}


void PCLRelay::pclCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_rcvd_msg){
    pcl_publisher_->publish(*pcl_rcvd_msg);
}

void PCLRelay::camCallBack(const sensor_msgs::msg::Image::SharedPtr cam_rcvd_msg){
    cam_publisher_->publish(*cam_rcvd_msg);
}   

void PCLRelay::toolRawInfoCallBack(const std_msgs::msg::Float64MultiArray::SharedPtr tool_raw_info_rcvd_msg){
    tool_raw_info_publisher_->publish(*tool_raw_info_rcvd_msg);
}

void PCLRelay::toolInfoCallBack(const lx_msgs::msg::ToolInfo::SharedPtr tool_info_rcvd_msg){
    tool_info_publisher_->publish(*tool_info_rcvd_msg);
}