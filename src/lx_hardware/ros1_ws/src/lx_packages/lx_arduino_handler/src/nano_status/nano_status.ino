/* Author: Dhruv Tyagi
 * Subscribers:
 *    - /hw_status_nano: [std_msgs::Bool] listen to heartbeat every 1.5 seconds
 *    - /op_status_nano: [std_msgs::Int32] current operation mode
 *    - /task_status_nano: [std_msgs::Int32] current task mode
 *    - /lock_status_nano: [std_msgs::Bool] current lock status
 *
 * - Source code for arduin nano to handle LED status indicators
 * - LED 1 
 *    Green: Power On
 * - LED 2
 *    Green: [Blinking] Heartbeat for ROS
 * - LED 3
 *    Green: Standby, White: Teleop, Blue: Autonomous
 * - LED 4
 *    Green: Idle, White: Nav, Blue: Exc, Red: DMP
 * - LED 5
 *    Red: Rover Locked
 * 
 * TODO
 * - 
 * */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>


//-------------------- Pins -------------------
// LED 1
#define LED_1g 13
// LED 2
#define LED_2g 12
// LED 3
#define LED_3r 11
#define LED_3g 10
#define LED_3b 9
// LED 4
#define LED_4r 6
#define LED_4g 5
#define LED_4b 3
// LED 5
#define LED_5r 2


//-------------- Global Variables -------------
// Default values
int val_1g = 0;
int val_2g = 255;
int val_3r = 255;
int val_3g = 255;
int val_3b = 255;
int val_4r = 255;
int val_4g = 255;
int val_4b = 255;
int val_5r = 255;
volatile unsigned long long last_hw_msg = millis();

// ---------------- ROS Setup -----------------
// ROS Node handle
ros::NodeHandle nh;

// Callbacks
void hwCB(const std_msgs::Bool& msg){
    // Hardware status indicated by 2nd LED
    if(msg.data){
      val_2g = 0;
    }
    else{
      val_2g = 255;
    }
    last_hw_msg = millis();
}
void opCB(const std_msgs::Int32& msg){
    // Operation mode indicated by 3rd LED
    switch(msg.data){
      case 0: // Standby mode : Green
        val_3r = 255;
        val_3g = 0;
        val_3b = 255;
        break;

      case 1: // Teleop mode : White
        val_3r = 0;
        val_3g = 0;
        val_3b = 0;
        break;

      case 2: // Autonomous mode : Blue
          val_3r = 255;
          val_3g = 255;
          val_3b = 0;
        break;

      default: // Default : Off
        val_3r = 255;
        val_3g = 255;
        val_3b = 255;
    }
}
void taskCB(const std_msgs::Int32& msg){
    // Task mode indicated by 4th LED
    switch(msg.data){
      case 0: // Idle mode : Green
        val_4r = 255;
        val_4g = 0;
        val_4b = 255;
        break;
      
      case 1: // Nav mode : White
        val_4r = 0;
        val_4g = 0;
        val_4b = 0;
        break;
      
      case 2: // Exc mode : Blue
        val_4r = 255;
        val_4g = 255;
        val_4b = 0;
        break;
      
      case 3: // Dmp mode : Red
        val_4r = 0;
        val_4g = 255;
        val_4b = 255;
        break;
      
      default: // Default : Off
        val_4r = 255;
        val_4g = 255;
        val_4b = 255;
    }
}
void lockCB(const std_msgs::Bool& msg){
    // Lock status indicated by 5th LED
    if(msg.data){
      val_5r = 0;
    }
    else{
      val_5r = 255;
    }
}


// ROS Variables
ros::Subscriber<std_msgs::Bool> hw_sub("/hw_status_nano", &hwCB);
ros::Subscriber<std_msgs::Int32> op_sub("/op_status_nano", &opCB);
ros::Subscriber<std_msgs::Int32> task_sub("/task_status_nano", &taskCB);
ros::Subscriber<std_msgs::Bool> lock_sub("/lock_status_nano", &lockCB);


//------------------- Setup ------------------
void setup() {
  // Initialize node
  nh.initNode();
  nh.subscribe(hw_sub);
  nh.subscribe(op_sub);
  nh.subscribe(task_sub);
  nh.subscribe(lock_sub);

  // LED 1 Output mode
  pinMode(LED_1g, OUTPUT);
  // LED 2 Output mode
  pinMode(LED_2g, OUTPUT);
  // LED 3 Output mode
  pinMode(LED_3r, OUTPUT);
  pinMode(LED_3g, OUTPUT);
  pinMode(LED_3b, OUTPUT);
  // LED 4 Output mode
  pinMode(LED_4r, OUTPUT);
  pinMode(LED_4g, OUTPUT);
  pinMode(LED_4b, OUTPUT);
  // LED 5 Output mode
  pinMode(LED_5r, OUTPUT);

  // LED 1 write value
  analogWrite(LED_1g, val_1g);
  // LED 2 write value
  analogWrite(LED_2g, val_2g);
  // LED 3 write value
  analogWrite(LED_3r, val_3r);
  analogWrite(LED_3g, val_3g);
  analogWrite(LED_3b, val_3b);
  // LED 4 write value
  analogWrite(LED_4r, val_4r);
  analogWrite(LED_4g, val_4g);
  analogWrite(LED_4b, val_4b);
  // LED 5 write value
  analogWrite(LED_5r, val_5r);

  while(nh.connected()==false){
    nh.spinOnce(); // Spin node 
    delay(50);
  }
  nh.loginfo("Nano setup completed");
}


//------------------- Loop -------------------
void loop() {
  // 2nd LED Heartbeat blink for 80 ms 
  if(millis() - last_hw_msg > 120 && val_2g != 255){
    val_2g = 255;
  }
  
  // LED 1 write value
  analogWrite(LED_1g, val_1g);
  // LED 2 write value
  analogWrite(LED_2g, val_2g);
  // LED 3 write value
  analogWrite(LED_3r, val_3r);
  analogWrite(LED_3g, val_3g);
  analogWrite(LED_3b, val_3b);
  // LED 4 write value
  analogWrite(LED_4r, val_4r);
  analogWrite(LED_4g, val_4g);
  analogWrite(LED_4b, val_4b);
  // LED 5 write value
  analogWrite(LED_5r, val_5r);

  nh.spinOnce(); // Spin node
  delay(50);
}
