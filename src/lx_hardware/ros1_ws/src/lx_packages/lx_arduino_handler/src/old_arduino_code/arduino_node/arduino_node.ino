// Modified from https: // github.com/micro-ROS/micro_ros_arduino/blob/1df47435f08b9609effaec9cb0cc99241ff9dc30/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
// Subscribers:
//    - /drum_cmd: Int32 PWM value between -255 to 255 
//    - /acc_cmd : Int32 PWM value between -255 to 255
// Publishers:
//    - /tool_raw_info: Float64MultiArray {drum_dticks_dt, acc_ticks, drum_current_read, acc_current_read}
// 
// The node locks actuation if the particular topic is not recieved for 3 seconds

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <cmath> // std::abs
#include <algorithm> // std::min and std::max
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <rcl_interfaces/msg/log.h>

//Define Pins
#define LED_PIN 13
#define CONN_PIN 12

//Drum
#define DRUM_DIR_PIN1 1
#define DRUM_DIR_PIN2 5
#define DRUM_PWM_PIN 1
#define DRUM_ENC_A 3
#define DRUM_ENC_B 2


//Linear Actuator
#define ACC_DIR_PIN1 1
#define ACC_DIR_PIN2 5
#define ACC_PWM_PIN 4
#define ACC_FEEDBACK_PIN 5

//Current Sensors
#define CURR_SENS1 6
#define CURR_SENS2 7

//Encoder Reading Variables
volatile double drum_ticks=0;
volatile double acc_ticks=0;
volatile unsigned int drum_read_curr, drum_read_prev;
volatile unsigned int acc_read_curr, acc_read_prev;
volatile unsigned int t_curr, t_prev;
volatile bool acc_dir = 1; //1 for forward, 0 for reverse 

//Interrupt Functions
void drum_position_interrupt() //Gives the position of the Motor wrt initial position (stored in ep)
{
  if (digitalRead(DRUM_ENC_A) == digitalRead(DRUM_ENC_B))  drum_ticks++;
  else drum_ticks--;
}

void acc_position_interrupt() //Gives the position of the Motor wrt initial position (stored in ep)
{
  if (acc_dir==1)  acc_ticks++;
  else acc_ticks--;
}

// Time Blocking dticks/dt functions
// void drum_dticks_dt()
// {
//   //Returns dticks/dt
//   drum_read_1=drum_ticks;
//   drum_t1=millis();
//   delay(10);
//   drum_read_2=drum_ticks;
//   drum_t2=millis();
//   double dticks_dt = ((double)drum_read_2-(double)drum_read_1)/((double)drum_t2-(double)drum_t1);
//   // if(abs(ans)>50)
//   //   ans = sgn(ans)*25;
//   return dticks_dt;
// }

// void acc_dticks_dt()
// {
//   acc_read_1=acc_ticks;
//   acc_t1=millis();
//   delay(10);
//   acc_read_2=acc_ticks;
//   acc_t2=millis();
//   double dticks_dt = ((double)acc_read_2-(double)acc_read_1)/((double)acc_t2-(double)acc_t1);
//   // if(abs(ans)>50)
//   //   ans = sgn(ans)*25;
//   return dticks_dt;
// }


/* MicroROS declarations */
// NUM_HANDLES must be updated to reflect total number of subscribers + publishers
#define NUM_HANDLES 4
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Declare microros objects
rclc_support_t support;
rcl_node_t node;
rcl_timer_t drum_timer, acc_timer, pub_timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t tool_raw_cmd_pub;
rcl_subscription_t drum_sub, acc_sub;
rcl_publisher_t publisher_log;

rcl_interfaces__msg__Log msgLog;
std_msgs__msg__Int32 drum_cmd;
std_msgs__msg__Int32 acc_cmd;
std_msgs__msg__Float64MultiArray pub_arr;
// std_msgs::msg::Float64MultiArray pub_arr;
bool micro_ros_init_successful;

// subscription variable and locks
int drum_pwm, acc_pwm;
unsigned int drum_msg_dT = 0, acc_msg_dT = 0;

//Debug Publisher
void publish_debug(String s)
{
  msgLog.msg.data = (char*) s.c_str();;
  msgLog.msg.size = strlen(msgLog.msg.data);
  RCSOFTCHECK(rcl_publish(&publisher_log, &msgLog, NULL));
}


/* ROS Callbacks */
void drum_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  drum_pwm = static_cast<int>(msg->data);
  drum_msg_dT = 0;
}

void acc_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  acc_pwm = static_cast<int>(msg->data);;
  acc_msg_dT = 0;
}

//Timer Callbacks
void drum_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer != NULL)
  {
    if (drum_msg_dT>3000) //if last message was within 3 seconds
    {
      drum_pwm = 0;
      // RCLCPP_INFO(node.get_logger(), "Locking Drum");
    }
    //write drum_pwm to pin
    if(drum_pwm>0)
    {
      digitalWrite(DRUM_DIR_PIN1, HIGH);
      digitalWrite(DRUM_DIR_PIN2, LOW);
    }
    else
    {
      digitalWrite(DRUM_DIR_PIN1, LOW);
      digitalWrite(DRUM_DIR_PIN2, HIGH);
    }
    analogWrite(DRUM_PWM_PIN, abs(drum_pwm));
    publish_debug("AnalogWrite Drum");
  }
}

void acc_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{ 
  (void)last_call_time;
  if (timer != NULL)
  {
    if (acc_msg_dT>3000) //if last message was within 3 seconds
    {
      acc_pwm = 0;
      // RCLCPP_INFO(node.get_logger(), "Locking Linear Actuator");
    }
    //write acc_pwm to pin
    if(acc_pwm>0)
    {
      digitalWrite(ACC_DIR_PIN1, HIGH);
      digitalWrite(ACC_DIR_PIN2, LOW);
      acc_dir = 1;
    }
    else
    {
      digitalWrite(ACC_DIR_PIN1, LOW);
      digitalWrite(ACC_DIR_PIN2, HIGH);
      acc_dir = 0;
    }
    analogWrite(ACC_PWM_PIN, abs(acc_pwm));
    publish_debug("AnalogWrite Linear Actuator");
  }
}

void pub_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{ 
  (void)last_call_time;
  if (timer != NULL)
  {
    // //Calculate dticks/dt for drum and linear actuator
    // acc_read_curr=acc_ticks; drum_read_curr = drum_ticks;
    // t_curr=millis();

    // //Publish Feedback from Encoders and Current Sensors using tool_raw_cmd_pub
    // pub_arr.data.size =4;
    // pub_arr.data.data[0] = (((double)drum_read_curr-(double)drum_read_prev)/((double)t_curr-(double)t_prev));

    // // pub_arr.data.data[1]= ((double)acc_read_curr-(double)acc_read_prev)/((double)t_curr-(double)t_prev);
    // pub_arr.data.data[1]= acc_ticks;
    // pub_arr.data.data[2]= analogRead(CURR_SENS1);
    // pub_arr.data.data[3]= analogRead(CURR_SENS2);
    // RCSOFTCHECK(rcl_publish(&tool_raw_cmd_pub, &pub_arr, NULL));

    // //Update previous values
    // drum_read_prev = drum_read_curr; acc_read_prev = acc_read_curr;
    // t_prev = t_curr;
    publish_debug("Publish Sensor Data");
          // publish_debug("Init Successful");
  }
}

/* MicroROS functions */
// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "arduino_actuator_interface_node", "", &support));

  // create publishers
  //rclc_publisher_init_default
  // RCCHECK(rclc_publisher_init_default(
  //     &tool_raw_cmd_pub,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
  //     "tool_raw_info"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_log,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
    "rosout")); //Debug publisher

  // // create subscriber
  // RCCHECK(rclc_subscription_init_default(
  //     &drum_sub,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //     "drum_cmd"));

  // RCCHECK(rclc_subscription_init_default(
  //     &acc_sub,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //     "acc_cmd"));

  // // create timers
  // const unsigned int drum_timer_period_ms = 50;
  // RCCHECK(rclc_timer_init_default(
  //     &drum_timer,
  //     &support,
  //     RCL_MS_TO_NS(drum_timer_period_ms),
  //     drum_timer_callback));
  
  // const unsigned int acc_timer_period_ms = 50;
  // RCCHECK(rclc_timer_init_default(
  //     &acc_timer,
  //     &support,
  //     RCL_MS_TO_NS(acc_timer_period_ms),
  //     acc_timer_callback));
  
  const unsigned int pub_timer_period_ms = 50;
  RCCHECK(rclc_timer_init_default(
      &pub_timer,
      &support,
      RCL_MS_TO_NS(pub_timer_period_ms),
      pub_timer_callback));

  // // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  // RCCHECK(rclc_executor_init(&executor, &support.context, NUM_HANDLES, &allocator));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &drum_timer));
  // RCCHECK(rclc_executor_add_timer(&executor, &acc_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &pub_timer));
  // RCCHECK(rclc_executor_add_subscription(&executor, &drum_sub, &drum_cmd, &drum_callback, ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_subscription(&executor, &acc_sub, &acc_cmd, &acc_callback, ON_NEW_DATA));

  // micro_ros_init_successful = true;

  msgLog.level = 2;
  msgLog.name.data = (char *)"Arduino Due ";
  msgLog.name.size = strlen(msgLog.name.data);
}

void destroy_entities()
{
  rcl_subscription_fini(&drum_sub, &node);
  rcl_subscription_fini(&acc_sub, &node);
  rcl_publisher_fini(&tool_raw_cmd_pub, &node);
  rcl_publisher_fini(&publisher_log, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&drum_timer);
  rcl_timer_fini(&acc_timer);
  rcl_timer_fini(&pub_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;
}

void setup()
{
  set_microros_transports();
  delay(2000);
  
  create_entities();

  //Setup Pins (With initial values)
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  // pinMode(CONN_PIN, OUTPUT); digitalWrite(CONN_PIN, LOW);

  // pinMode(DRUM_PWM_PIN, OUTPUT); analogWrite(DRUM_PWM_PIN, HIGH);
  // pinMode(DRUM_ENC_A, INPUT);
  // pinMode(DRUM_ENC_B, INPUT);
  // pinMode(DRUM_DIR_PIN1, OUTPUT); digitalWrite(DRUM_DIR_PIN1, LOW);
  // pinMode(DRUM_DIR_PIN2, OUTPUT); digitalWrite(DRUM_DIR_PIN2, HIGH);

  // //Linear Actuator
  // pinMode(ACC_PWM_PIN, OUTPUT); analogWrite(DRUM_PWM_PIN, HIGH);
  // pinMode(ACC_DIR_PIN1, OUTPUT); digitalWrite(ACC_DIR_PIN1, LOW);
  // pinMode(ACC_DIR_PIN2, OUTPUT); digitalWrite(ACC_DIR_PIN2, HIGH);
  // pinMode(ACC_FEEDBACK_PIN, INPUT);

  // //Current Sensors
  // pinMode(CURR_SENS1, INPUT);
  // pinMode(CURR_SENS2, INPUT);

  // //Interrupts to read encoder data
  // attachInterrupt(digitalPinToInterrupt(DRUM_ENC_A), drum_position_interrupt, RISING);//Interrupt activates when a gets rising edge
  // attachInterrupt(digitalPinToInterrupt(ACC_FEEDBACK_PIN), acc_position_interrupt, RISING);//Interrupt activates when pin gets rising edge

  micro_ros_init_successful = false;

  // Store default values in _prev variables
  drum_read_prev = 0;
  acc_read_prev = 0;
  t_prev = millis();
}

uint32_t delay_ms = 200; // short delay to blink the red LED while trying to connect
void loop()
{
  // // Keep trying to connect by pinging the MicroROS agent
  if (RMW_RET_OK == rmw_uros_ping_agent(50, 2))
  {
    // Use flag to see if entities need to be created
    if (!micro_ros_init_successful)
    {
      create_entities();
    }
    else
    {
      //Increment timer values 
      drum_msg_dT += 5;
      acc_msg_dT += 5;
      // Main loop to run the MicroROS node
      digitalWrite(CONN_PIN, HIGH); // Green LED on
      digitalWrite(LED_PIN, LOW); // Red LED off
      // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    }
  }
  else {
    // Destroy entities if there is not connection to the agent
    if (micro_ros_init_successful)
    {
      destroy_entities();
      digitalWrite(CONN_PIN, LOW); // Green LED off
    }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink red LED while trying to connect
    delay(delay_ms);
  }
}