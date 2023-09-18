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

    }

private:
    geometry_msgs::msg::TransformStamped eigen_transform_prism_baselink;
    // sensor_msgs::msg::Imu eigen_transform_imu_baselink;
    bool got_transform = false;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    double x_off = 10, y_off = -13, z_off = 11;
    bool set_offset = false;
    double yaw = 0;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){

        // try 
        // {
        //     std::cout<<"Looking for transform"<<std::endl;
        //     // Get transform matrix from total_station_prism to base_link
        //     tf2_ros::Buffer tf_buffer(this->get_clock());
        //     tf2_ros::TransformListener tf_listener(tf_buffer);
        //     this->eigen_transform_imu_baselink = tf_buffer.lookupTransform("base_link", "imu", tf2::TimePointZero, tf2::durationFromSec(0.5)); //imu to base link
        // } 
        // catch (tf2::TransformException& ex) 
        // {
        //     std::cout<<"No transform found, publishing directly"<<std::endl;
        // }      
            
        // sensor_msgs::msg::Imu imu_map_msg;
        // tf2::doTransform(*msg, imu_map_msg, this->eigen_transform_imu_baselink);
        // imu_map_msg.header.frame_id = "map";
        // imu_map_msg.header.stamp = msg->header.stamp;
        yaw = msg->orientation.z;
    }


    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // if(set_offset==false)
        // {
        //     x_off = -7;
        //     y_off = -10;
        //     z_off = 10;
        //     // std::cout<<"Setting offset: "<<x_off<<", "<<y_off<<", "<<z_off<<std::endl;
        //     set_offset = true;
        // }
        if(this->got_transform == false)
        {
            try 
            {
                std::cout<<"Looking for transform"<<std::endl;
                // Get transform matrix from total_station_prism to base_link
                tf2_ros::Buffer tf_buffer(this->get_clock());
                tf2_ros::TransformListener tf_listener(tf_buffer);
                this->eigen_transform_prism_baselink = tf_buffer.lookupTransform("base_link", "total_station_prism", tf2::TimePointZero, tf2::durationFromSec(0.5)); //primm to base link
                this->got_transform = true;
            } 
            catch (tf2::TransformException& ex) 
            {
                std::cout<<"No transform found, publishing directly"<<std::endl;
                msg->header.frame_id = "map";
                msg->pose.pose.position.x += x_off;
                msg->pose.pose.position.y += y_off;
                msg->pose.pose.position.z += z_off;
                pose_pub_->publish(*msg);
            }
        }
        if (this->got_transform == true)
        {   
            // std::cout<<"Transform found, publishing"<<std::endl;
            geometry_msgs::msg::PoseWithCovarianceStamped pose_map_msg;
            msg->pose.pose.position.x += x_off;
            msg->pose.pose.position.y += y_off;
            msg->pose.pose.position.z += z_off;
            tf2::doTransform(*msg, pose_map_msg, this->eigen_transform_prism_baselink);
            pose_map_msg.header.frame_id = "map";
            pose_map_msg.header.stamp = msg->header.stamp;
            // pose_map_msg.pose.pose.position.x += ;
            // pose_map_msg.pose.pose.position.y += y_off;
            // pose_map_msg.pose.pose.position.z += z_off;
            pose_map_msg.pose.pose.orientation.z = yaw;
            pose_pub_->publish(pose_map_msg);
            std::cout<<pose_map_msg.pose.pose.orientation.z<<" "<<yaw<<std::endl;
        }
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
