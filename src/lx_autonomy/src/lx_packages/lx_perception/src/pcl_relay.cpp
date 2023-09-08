#include "lx_perception/pcl_relay.hpp"

PCLRelay::PCLRelay(): Node("pcl_relay_node"){
    // Set up subscriptions & publishers
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "PCL Relay initialized");
}

void PCLRelay::setupCommunications(){
    pcl_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points", 1 , 
                            std::bind(&PCLRelay::pclCallBack, this, std::placeholders::_1));
    
    relay_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_relay", 1);

    // Set the QoS settings best effort
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();


    // Create the subscriber
    cam_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", qos, std::bind(&PCLRelay::camCallBack, this, std::placeholders::_1));

    // Create the publisher
    cam_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("cam_relay", qos);
}

void PCLRelay::pclCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_rcvd_msg){
    auto pcl_pub_msg = sensor_msgs::msg::PointCloud2();
    
    pcl_pub_msg.header.frame_id = pcl_rcvd_msg->header.frame_id;
    pcl_pub_msg.header.stamp = pcl_rcvd_msg->header.stamp;
    pcl_pub_msg.height = pcl_rcvd_msg->height;
    pcl_pub_msg.width = pcl_rcvd_msg->width;
    pcl_pub_msg.fields = pcl_rcvd_msg->fields;
    pcl_pub_msg.is_bigendian = pcl_rcvd_msg->is_bigendian;
    pcl_pub_msg.point_step = pcl_rcvd_msg->point_step;
    pcl_pub_msg.row_step = pcl_rcvd_msg->row_step;
    pcl_pub_msg.data = pcl_rcvd_msg->data;
    pcl_pub_msg.is_dense = pcl_rcvd_msg->is_dense;

    relay_publisher_->publish(pcl_pub_msg);
}

void PCLRelay::camCallBack(const sensor_msgs::msg::Image::SharedPtr cam_rcvd_msg){
    // print some info about the image
    cam_publisher_->publish(*cam_rcvd_msg);
}   