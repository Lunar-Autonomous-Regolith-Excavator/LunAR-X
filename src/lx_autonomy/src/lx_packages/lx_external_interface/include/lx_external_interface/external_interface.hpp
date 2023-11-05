#ifndef EXTERNAL_INTERFACE_H
#define EXTERNAL_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lx_msgs/msg/rover_command.hpp"
#include "lx_library/lx_utils.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "lx_msgs/srv/switch.hpp"
#include "lx_msgs/msg/node_diagnostics.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lx_msgs/action/calibrate_imu.hpp"


class ExternalInterface: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        unsigned int diagnostic_pub_period_ = 1;
        using CalibrateImu = lx_msgs::action::CalibrateImu;
        // Time
        rclcpp::TimerBase::SharedPtr diagnostic_pub_timer_;
        rclcpp::TimerBase::SharedPtr rover_lock_timer_;
        rclcpp::Time guide_debounce_timer_;
        rclcpp::Time start_debounce_timer_;
        rclcpp::Time back_debounce_timer_;
        rclcpp::Time a_debounce_timer_;
        rclcpp::Time b_debounce_timer_;
        rclcpp::Time y_debounce_timer_;
        // Track joystick states
        sensor_msgs::msg::Joy joy_last_state_ = sensor_msgs::msg::Joy();
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
        std::thread rover_control_pub_thread_;
        // Clients
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_params_client_;
		rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_params_client_;
        rclcpp::Client<lx_msgs::srv::Switch>::SharedPtr map_switch_client_;
        rclcpp_action::Client<CalibrateImu>::SharedPtr calibrate_imu_action_client_;
        // Publishers
        rclcpp::Publisher<lx_msgs::msg::NodeDiagnostics>::SharedPtr diagnostic_publisher_;
        rclcpp::Publisher<lx_msgs::msg::RoverCommand>::SharedPtr rover_teleop_publisher_;
        // Parameter handling
        struct lock_struct rover_soft_lock_;
        OpModeEnum current_rover_op_mode_ = OpModeEnum::STANDBY;
        TaskModeEnum current_rover_task_mode_ = TaskModeEnum::IDLE;
        float mob_lin_vel_ = 0.0; 
        float mob_ang_vel_ = 0.0;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> mob_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> act_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> op_mode_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> task_mode_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> lin_mob_vel_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> ang_mob_vel_param_cb_handle_;
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
        * Get starting values of global parameters
        * */
        void getParams();

        /*
        * Callback function for starting values of global parameters
        * */
        void paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture );

        /*
        * Argument(s): 
        *   - joystick message
        * 
        * Callback function for /joy topic published by the joystick
        * */
        void joyCallBack(const sensor_msgs::msg::Joy::SharedPtr );

        /*
        * Due to an unclear issue, the mobility and actuation lock statuses flip. This function will correct for this undesirable behaviour 
        * */
        void lockCheck();
        
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
        * Argument(s):
        *   - joystick message
        * 
        * Publish rover teleop command using the joystick input
        * */
        void passRoverTeleopCmd(const sensor_msgs::msg::Joy::SharedPtr );

        /*
        * Argument(s):
        *   - joystick trigger value
        *
        * Remap joystick trigger value to [0 to 1] for drum command
        * */
        double remapTrig(float );
        
        /*
        * Call service to switch on or off mapping
        * */
        void callStartMappingSwitch(bool );

        /*
        * Callback function for map switch service
        * */
        void mapSwitchCB(rclcpp::Client<lx_msgs::srv::Switch>::SharedFuture );

        /*
        * Call localization calibration action
        * */
        void callLocalizationCalibration();

        /*
        * Diagnostic heartbeat published at a fixed rate
        * */
        void diagnosticPublish();
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