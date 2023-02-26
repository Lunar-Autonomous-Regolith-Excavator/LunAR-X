#ifndef EXTERNAL_INTERFACE_H
#define EXTERNAL_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "lx_msgs/msg/rover_lock.hpp"
#include "lx_msgs/msg/rover_op_mode.hpp"
#include "lx_msgs/msg/rover_teleop.hpp"
#include "lx_utils/lx_utils.hpp"

#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"


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
        // rclcpp::Publisher<lx_msgs::msg::RoverLock>::SharedPtr rover_lock_publisher_;
        rclcpp::Publisher<lx_msgs::msg::RoverTeleop>::SharedPtr rover_teleop_publisher_;

        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> mob_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> act_param_cb_handle_;
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_params_client_;

        // Functions

        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Argument(s): 
        *   - joystick message
        * 
        * Callback function for /joy topic published by the joystick
        * */
        void joyCallBack(const sensor_msgs::msg::Joy::SharedPtr );

        /*
        * Argument(s):
        *   - joystick message
        * 
        * Handle co-ordinating the publishing of any changes to rover mode/lock status
        * */
        void roverControlPublish(const sensor_msgs::msg::Joy::SharedPtr );

        /*
        * Set rover lock status to true and call to publish
        * */
        void lockRover();

        /*
        * Publish required lock status
        * */
        void switchLockStatus();

        /*
        * Automatically set rover lock status to true if no communication
        * received from the joystick for 3 seconds
        * */
        void activeLock();

        /*
        * Publish required rover operation mode
        * */
        void switchRoverOpMode();

        /*
        * Argument(s):
        *   - joystick message
        * 
        * Save last received joystick state. Required to know changes in state.
        * */
        void setLastJoyState(const sensor_msgs::msg::Joy::SharedPtr );

        /*
        * TODO
        * */
        void passRoverTeleopCmd(const sensor_msgs::msg::Joy::SharedPtr );

        // TODO
        void setupParams();

    public:
        // Functions
        /*
        * Constructor
        * */
        ExternalInterface();

        /*
        * Destructor
        * */
        ~ExternalInterface(){}
};

#endif