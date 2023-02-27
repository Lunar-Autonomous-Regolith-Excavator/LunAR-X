#ifndef EXTERNAL_INTERFACE_H
#define EXTERNAL_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lx_msgs/msg/rover_teleop.hpp"
#include "lx_utils/lx_utils.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"


class ExternalInterface: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        // Time
        rclcpp::TimerBase::SharedPtr rover_lock_timer_;
        rclcpp::Time guide_debounce_timer_;
        rclcpp::Time start_debounce_timer_;
        rclcpp::Time back_debounce_timer_;
        // Track joystick states
        sensor_msgs::msg::Joy joy_last_state_ = sensor_msgs::msg::Joy();
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
        std::thread rover_control_pub_thread_;
        // Publishers
        rclcpp::Publisher<lx_msgs::msg::RoverTeleop>::SharedPtr rover_teleop_publisher_;
        // Parameter handling
        struct lock_struct rover_soft_lock_;
        OpModeEnum current_rover_op_mode_ = OpModeEnum::STANDBY;
        TaskModeEnum current_rover_task_mode_ = TaskModeEnum::IDLE;
        float mob_lin_vel_ = 0.5;
        float mob_ang_vel_ = 0.1;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> mob_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> act_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> op_mode_param_cb_handle_;
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_params_client_;
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Set up tracking of global parameters
        * */
        void setupParams();

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
        * Argument(s):
        *   - desired mobility lock value
        *   - desired actuation lock value
        * 
        * Publish required lock status
        * */
        void switchRoverLockStatus(bool, bool);

        /*
        * Automatically set rover lock status to true if no communication
        * received from the joystick for 3 seconds
        * */
        void activeLock();

        /*
        * Argument(s):
        *   - desired operation mode
        * 
        * Publish required rover operation mode
        * */
        void switchRoverOpMode(OpModeEnum );

        /*
        * Argument(s):
        *   - desired task mode
        * 
        * Publish required rover task mode
        * */
        void switchRoverTaskMode(TaskModeEnum );

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
        
        // --------------------------------------

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