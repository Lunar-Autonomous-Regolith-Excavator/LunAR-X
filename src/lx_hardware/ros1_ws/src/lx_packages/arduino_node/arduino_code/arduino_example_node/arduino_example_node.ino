#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//Logging
#include <rcl_interfaces/msg/log.h>
rcl_publisher_t publisher_log;
rcl_interfaces__msg__Log msgLog;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
    publish_debug("Publish Sensor Data");
  }
}

void publish_debug(String s)
{
  msgLog.msg.data = (char*) s.c_str();;
  msgLog.msg.size = strlen(msgLog.msg.data);
  RCSOFTCHECK(rcl_publish(&publisher_log, &msgLog, NULL));
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_log,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
    "rosout")); //Debug publisher

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;

  
  msgLog.level = 2;
  msgLog.name.data = (char *)"Arduino Due ";
  msgLog.name.size = strlen(msgLog.name.data);

  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);

}

void loop() {
  if (RMW_RET_OK == rmw_uros_ping_agent(50, 2))
  {
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
  else
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink red LED while trying to connect
    delay(200);
  }
  
  // Serial.println("In loop");


    // // // Keep trying to connect by pinging the MicroROS agent
  // if (RMW_RET_OK == rmw_uros_ping_agent(50, 2))
  // {
  //   // Use flag to see if entities need to be created
  //   if (!micro_ros_init_successful)
  //   {
  //     create_entities();
  //   }
  //   else
  //   {
      //Increment timer values 
      // drum_msg_dT += 5;
      // acc_msg_dT += 5;
      // // Main loop to run the MicroROS node
      // digitalWrite(CONN_PIN, HIGH); // Green LED on
      // digitalWrite(LED_PIN, LOW); // Red LED off
      // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  //   }
  // }
  // else {
  //   // Destroy entities if there is not connection to the agent
  //   if (micro_ros_init_successful)
  //   {
  //     destroy_entities();
  //     digitalWrite(CONN_PIN, LOW); // Green LED off
  //   }
  //   digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink red LED while trying to connect
  //   delay(delay_ms);
  // }
}
