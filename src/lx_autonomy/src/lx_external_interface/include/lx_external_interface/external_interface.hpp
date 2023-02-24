#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "lx_msgs/msg/rover_lock.hpp"
#include "lx_msgs/msg/rover_op_mode.hpp"
#include "lx_msgs/msg/rover_teleop.hpp"
#include "lx_utils/lx_utils.hpp"


class ExternalInterface: public rclcpp::Node
{
    private:
        // Variables & pointers
        rclcpp::TimerBase::SharedPtr rover_lock_timer_;
        std::thread rover_control_pub_thread_;
        struct lock_struct rover_soft_lock_;
        OpModeEnum current_rover_op_mode_;
        sensor_msgs::msg::Joy joy_last_state_ = sensor_msgs::msg::Joy();
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
        rclcpp::Publisher<lx_msgs::msg::RoverOpMode>::SharedPtr rover_mode_publisher_;
        rclcpp::Publisher<lx_msgs::msg::RoverLock>::SharedPtr rover_lock_publisher_;
        rclcpp::Publisher<lx_msgs::msg::RoverTeleop>::SharedPtr rover_teleop_publisher_;

        // Functions
        void setupCommunications();
        void joyCallBack(const sensor_msgs::msg::Joy::SharedPtr );
        void roverControlPublish(const sensor_msgs::msg::Joy::SharedPtr );
        void lockRover();
        void unlockRover();
        void switchRoverOpMode();
        void setLastJoyState(const sensor_msgs::msg::Joy::SharedPtr );
        void switchRoverLockStatus();
        void activeLock();
        void passRoverTeleopCmd(const sensor_msgs::msg::Joy::SharedPtr );

    public:
        // Functions
        ExternalInterface();
        ~ExternalInterface(){}
};