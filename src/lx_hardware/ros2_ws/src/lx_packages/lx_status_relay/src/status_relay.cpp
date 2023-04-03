#include "lx_status_relay/status_relay.hpp"

StatusRelay::StatusRelay(): Node("status_relay_node"){
    // Set up timers
    setupTimers();
    
    // Set up publishers & clients
    setupCommunications();

    RCLCPP_INFO(this->get_logger(), "Status Relay initialized");
}

void StatusRelay::setupTimers(){
  // Get parameters timer, call get parameters service every 0.5 sec
  get_params_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&StatusRelay::getParameters, this));
  
  // Hardware publish timer, publish heartbeat every 1.5 sec
  hw_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(1500), std::bind(&StatusRelay::pubHWStatus, this));
}

void StatusRelay::setupCommunications(){
  // Publishers
  hw_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("hw_status_nano", 10);
  op_status_publisher_ = this->create_publisher<std_msgs::msg::Int32>("op_status_nano", 10);
  task_status_publisher_ = this->create_publisher<std_msgs::msg::Int32>("task_status_nano", 10);
  lock_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("lock_status_nano", 10);

  // Clients
  get_params_client_ = this->create_client<rcl_interfaces::srv::GetParameters>("/param_server_node/get_parameters");
}

void StatusRelay::pubHWStatus(){
    // Make hw status message
    auto hw_msg = std_msgs::msg::Bool();
    hw_msg.data = true;

    // Publish
    hw_status_publisher_->publish(hw_msg);
}

void StatusRelay::pubLockStatus(bool lock_val){
    // Make lock status message
    auto lock_msg = std_msgs::msg::Bool();
    lock_msg.data = lock_val;

    // Publish
    lock_status_publisher_->publish(lock_msg);
}

void StatusRelay::pubOpModeStatus(int op_mode){
    // Make op mode message
    auto op_msg = std_msgs::msg::Int32();
    op_msg.data = op_mode;

    // Publish
    op_status_publisher_->publish(op_msg);
}

void StatusRelay::pubTaskStatus(int task_mode){
    // Make task mode message
    auto task_msg = std_msgs::msg::Int32();
    task_msg.data = task_mode;

    // Publish
    task_status_publisher_->publish(task_msg);
}

void StatusRelay::getParameters(){
  // Wait for parameter server
  while(!get_params_client_->wait_for_service(std::chrono::seconds(2))){
      RCLCPP_INFO(this->get_logger(), "Could not contact param server");
      return;
  }

  // Request for parameter values
  auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  get_request->names = {"rover.mobility_lock", "rover.actuation_lock", "rover.op_mode", "rover.task_mode"};

  // Send request
  auto param_result_ = get_params_client_->async_send_request(get_request,std::bind(&StatusRelay::paramCB, this, std::placeholders::_1));

}

void StatusRelay::paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future){
    auto status = future.wait_for(std::chrono::milliseconds(100));
    // If parameters were returned
    if (status == std::future_status::ready) {
      // Call lock status publisher
      bool mob_lock_val = future.get()->values.at(0).bool_value;
      bool act_lock_val = future.get()->values.at(1).bool_value;
      if(mob_lock_val && act_lock_val){
        pubLockStatus(true);
      }
      else{
        pubLockStatus(false);
      }

      // Call op mode publisher
      int op_int = future.get()->values.at(2).integer_value;
      pubOpModeStatus(op_int);

      // Call task mode publisher
      int task_int = future.get()->values.at(3).integer_value;
      pubTaskStatus(task_int);

    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}