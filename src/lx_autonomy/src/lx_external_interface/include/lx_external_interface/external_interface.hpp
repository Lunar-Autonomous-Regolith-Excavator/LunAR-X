#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "lx_msgs/msg/rover_lock.hpp"
#include "lx_msgs/msg/rover_op_mode.hpp"
#include "lx_msgs/msg/rover_teleop.hpp"

struct lock_struct{
    bool mobility_lock;
    bool actuation_lock;
};

class ExternalInterface: public rclcpp::Node
{
    private:
        // Variables & pointers
        std::thread rover_control_pub_thread_;
        struct lock_struct rover_soft_lock_;
        bool current_rover_op_mode_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
        rclcpp::Publisher<lx_msgs::msg::RoverOpMode>::SharedPtr rover_mode_publisher_;
        rclcpp::Publisher<lx_msgs::msg::RoverLock>::SharedPtr rover_lock_publisher_;
        rclcpp::Publisher<lx_msgs::msg::RoverTeleop>::SharedPtr rover_teleop_publisher_;

        // Functions
        void setupCommunications();
        void joyCallBack(const sensor_msgs::msg::Joy::SharedPtr );
        void roverControlPublish(const sensor_msgs::msg::Joy::SharedPtr );

    public:
        // Functions
        ExternalInterface();
        ~ExternalInterface(){}
};