#include "lx_mapping/pcl_relay.hpp"

PCLRelay::PCLRelay(): Node("pcl_relay_node"){
    // Set up subscriptions & publishers
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "PCL Relay initialized");
}

void PCLRelay::setupCommunications(){
    // Subscribers with best effort QoS
    auto qos = rclcpp::SensorDataQoS();
    pcl_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points", qos , 
                            std::bind(&PCLRelay::pclCallBack, this, std::placeholders::_1));
    // aligned subscriber with best effort qos
    aligned_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/aligned_depth_to_color/image_raw", qos, 
                            std::bind(&PCLRelay::alignedCallBack, this, std::placeholders::_1));
    
    color_image_raw_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", qos,
                            std::bind(&PCLRelay::colorImageRawCallBack, this, std::placeholders::_1));

    cam_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", qos,
                            std::bind(&PCLRelay::camInfoCallBack, this, std::placeholders::_1));

    // Publisher
    relay_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_relay", 10);
    aligned_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("aligned_relay", 10 );
    color_image_raw_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("color_image_raw_relay", 10);
    cam_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("cam_info_relay", 10);

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

void PCLRelay::alignedCallBack(const sensor_msgs::msg::Image::SharedPtr aligned_rcvd_msg){
    auto aligned_pub_msg = sensor_msgs::msg::Image();
    aligned_pub_msg.header.frame_id = aligned_rcvd_msg->header.frame_id;
    aligned_pub_msg.header.stamp = aligned_rcvd_msg->header.stamp;
    aligned_pub_msg.height = aligned_rcvd_msg->height;
    aligned_pub_msg.width = aligned_rcvd_msg->width;
    aligned_pub_msg.encoding = aligned_rcvd_msg->encoding;
    aligned_pub_msg.is_bigendian = aligned_rcvd_msg->is_bigendian;
    aligned_pub_msg.step = aligned_rcvd_msg->step;
    aligned_pub_msg.data = aligned_rcvd_msg->data;

    aligned_publisher_->publish(aligned_pub_msg);
}

void PCLRelay::colorImageRawCallBack(const sensor_msgs::msg::Image::SharedPtr color_image_raw_rcvd_msg){
    auto color_image_raw_pub_msg = sensor_msgs::msg::Image();

    color_image_raw_pub_msg.header.frame_id = color_image_raw_rcvd_msg->header.frame_id;
    color_image_raw_pub_msg.header.stamp = color_image_raw_rcvd_msg->header.stamp;
    color_image_raw_pub_msg.height = color_image_raw_rcvd_msg->height;
    color_image_raw_pub_msg.width = color_image_raw_rcvd_msg->width;
    color_image_raw_pub_msg.encoding = color_image_raw_rcvd_msg->encoding;
    color_image_raw_pub_msg.is_bigendian = color_image_raw_rcvd_msg->is_bigendian;
    color_image_raw_pub_msg.step = color_image_raw_rcvd_msg->step;
    color_image_raw_pub_msg.data = color_image_raw_rcvd_msg->data;

    color_image_raw_publisher_->publish(color_image_raw_pub_msg);
}

void PCLRelay::camInfoCallBack(const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_rcvd_msg){
    auto cam_info_pub_msg = sensor_msgs::msg::CameraInfo();

    cam_info_pub_msg.header.frame_id = cam_info_rcvd_msg->header.frame_id;
    cam_info_pub_msg.header.stamp = cam_info_rcvd_msg->header.stamp;
    cam_info_pub_msg.height = cam_info_rcvd_msg->height;
    cam_info_pub_msg.width = cam_info_rcvd_msg->width;
    cam_info_pub_msg.distortion_model = cam_info_rcvd_msg->distortion_model;
    cam_info_pub_msg.d = cam_info_rcvd_msg->d;
    cam_info_pub_msg.k = cam_info_rcvd_msg->k;
    cam_info_pub_msg.r = cam_info_rcvd_msg->r;
    cam_info_pub_msg.p = cam_info_rcvd_msg->p;
    cam_info_pub_msg.binning_x = cam_info_rcvd_msg->binning_x;
    cam_info_pub_msg.binning_y = cam_info_rcvd_msg->binning_y;
    cam_info_pub_msg.roi = cam_info_rcvd_msg->roi;

    cam_info_publisher_->publish(cam_info_pub_msg);
}