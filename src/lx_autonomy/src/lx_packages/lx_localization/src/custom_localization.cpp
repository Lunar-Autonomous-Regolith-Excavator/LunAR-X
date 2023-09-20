#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.h>
#include <sensor_msgs/msg/imu.hpp>

// For tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RemapNode : public rclcpp::Node
{
public:
    RemapNode() : Node("custom_localization")
    {
        // Subscribe to the "odom" topic and remap it to "my_odom"
        auto qos = rclcpp::SensorDataQoS();

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/total_station_pose_map", qos);
        
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/total_station_prism", qos,
                                        std::bind(&RemapNode::pose_callback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/vectornav/imu", qos,
                                        std::bind(&RemapNode::imu_callback, this, std::placeholders::_1));

        final_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/odometry/ekf_odom_node", qos);

    }

private:
    geometry_msgs::msg::TransformStamped eigen_transform_prism_baselink;
<<<<<<< HEAD
    sensor_msgs::msg::Imu eigen_transform_imu_baselink;
    bool got_transform = false;
=======
    geometry_msgs::msg::TransformStamped eigen_transform_imu_baselink;
    // sensor_msgs::msg::Imu eigen_transform_imu_baselink;
    bool got_prism_transform = false;
    bool got_imu_transform = false;
>>>>>>> 9fa75209e5b06d912be07632f794f3acca9854ac
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr final_pose_pub_;
    double x_off = 10, y_off = -13, z_off = 11;
    bool set_offset = false;
    geometry_msgs::msg::PoseWithCovarianceStamped imu_pose_msg;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
<<<<<<< HEAD

        try 
        {
            std::cout<<"Looking for transform"<<std::endl;
            // Get transform matrix from total_station_prism to base_link
            tf2_ros::Buffer tf_buffer(this->get_clock());
            tf2_ros::TransformListener tf_listener(tf_buffer);
            // this->eigen_transform_imu_baselink = tf_buffer.lookupTransform("base_link", "imu", tf2::TimePointZero, tf2::durationFromSec(0.5)); //imu to base link
        } 
        catch (tf2::TransformException& ex) 
        {
            std::cout<<"No transform found, publishing directly"<<std::endl;
        }      
            
        sensor_msgs::msg::Imu imu_map_msg;
        // tf2::doTransform(*msg, imu_map_msg, this->eigen_transform_imu_baselink);
        
        imu_map_msg.header.frame_id = "map";
        imu_map_msg.header.stamp = msg->header.stamp;

        yaw = msg->orientation.z;
=======
        imu_pose_msg.header.frame_id = "odom";
        imu_pose_msg.header.stamp = msg->header.stamp;
        imu_pose_msg.pose.pose.orientation = msg->orientation;   
>>>>>>> 9fa75209e5b06d912be07632f794f3acca9854ac
    }

    void check_for_prism_transform()
    {
        std::cout<<"Looking for prism transform"<<std::endl;
        // Get transform matrix from total_station_prism to base_link
        try {
            tf2_ros::Buffer tf_buffer(this->get_clock());
            tf2_ros::TransformListener tf_listener(tf_buffer);
            this->eigen_transform_prism_baselink = tf_buffer.lookupTransform("base_link", "total_station_prism", tf2::TimePointZero, tf2::durationFromSec(0.5)); //prism to base link
            this->got_prism_transform = true;
        }
        catch (tf2::TransformException& ex) 
        {
            std::cout<<"No transform found"<<std::endl;
            this->got_prism_transform = false;
        }
    }

    void check_for_imu_transform()
    {
        std::cout<<"Looking for IMU transform"<<std::endl;
        // Get transform matrix from imu to base_link
        try {
            tf2_ros::Buffer tf_buffer(this->get_clock());
            tf2_ros::TransformListener tf_listener(tf_buffer);
            this->eigen_transform_imu_baselink = tf_buffer.lookupTransform("base_link", "vectornav", tf2::TimePointZero, tf2::durationFromSec(0.5)); //imu to base link
            this->got_imu_transform = true;
        }
        catch (tf2::TransformException& ex) 
        {
            std::cout<<"No transform found"<<std::endl;
            this->got_imu_transform = false;
        }
    }


    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!this->got_prism_transform) check_for_prism_transform();
        if (!this->got_imu_transform) check_for_imu_transform();

        geometry_msgs::msg::PoseWithCovarianceStamped pose_map_msg;
        pose_map_msg.header.frame_id = "odom";
        pose_map_msg.header.stamp = msg->header.stamp;
        // msg->pose.pose.position.x += x_off;
        // msg->pose.pose.position.y += y_off;
        // msg->pose.pose.position.z += z_off;
        
        if (this->got_prism_transform) tf2::doTransform(*msg, pose_map_msg, this->eigen_transform_prism_baselink);
        if (this->got_imu_transform) tf2::doTransform(*msg, imu_pose_msg, this->eigen_transform_imu_baselink);

        pose_map_msg.pose.pose.orientation = imu_pose_msg.pose.pose.orientation;
        this->final_pose_pub_->publish(pose_map_msg);
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RemapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
