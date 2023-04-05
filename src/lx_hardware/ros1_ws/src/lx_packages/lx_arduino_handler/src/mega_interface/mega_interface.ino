/* Author: Vibhakar Mohta
 * Subscribers:
 *    - /drum_cmd: [std_msgs::Int32] PWM value between -255 to 255 
 *    - /acc_cmd : [std_msgs::Int32] PWM value between -255 to 255
 * Publishers:
 *    - /tool_raw_info: [std_msgs::Float64MultiArray] {drum_dticks_dt, acc_ticks, drum_current_read, acc_current_read}
 *
 * - Source code for arduino mega to handle action and encoder feedback
 * - The node locks actuation if the particular topic is not recieved for 3 seconds
 * 
 * TODO
 * - 
 * */


// #define USE_USBCON
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

//--------------------Pins--------------------//
#define LED_PIN 13
// #define CONN_PIN 12

//Drum
#define DRUM_DIR_PIN1 4
// #define DRUM_DIR_PIN2 3
#define DRUM_PWM_PIN 5
#define DRUM_ENC_A 2
#define DRUM_ENC_B 3

//Linear Actuator
#define ACC_DIR_PIN1 11
// #define ACC_DIR_PIN2 8
#define ACC_PWM_PIN 12
#define ACC_FEEDBACK_PIN 18

//Current Sensors
#define CURR_SENS_DRUM A0
#define CURR_SENS_ACC A1

//------------Global Variables------------//
//Encoder Reading Variables
long long int drum_ticks=0;
volatile double acc_ticks=0;
volatile unsigned int drum_read_curr, drum_read_prev;
volatile unsigned int acc_read_curr, acc_read_prev;
volatile unsigned int t_curr, t_prev;
volatile bool acc_dir = 1; //1 for forward, 0 for reverse 
volatile bool node_started = false; //1 for forward, 0 for reverse 

//Callback Data Variables
int drum_pwm=0, acc_pwm=0;
unsigned int drum_msg_dT = 0, acc_msg_dT = 0;

//------------Functions------------//
//Interrupt Functions
void drum_position_interrupt() //Gives the position of the Motor wrt initial position (stored in ep)
{
  if(node_started==false) return;
  if (digitalRead(DRUM_ENC_A) == digitalRead(DRUM_ENC_B))  drum_ticks++;
  else drum_ticks--;
}

void acc_position_interrupt() //Gives the position of the Motor wrt initial position (stored in ep)
{
  if(node_started==false) return;
  if (acc_dir==1)  acc_ticks += 1.0; 
  else             acc_ticks -= 1.0;
}

ros::NodeHandle nh;

void drumCmdCB(const std_msgs::Int32& msg)
{
    drum_pwm = msg.data;
    drum_msg_dT = 0;
}

void accCmdCB(const std_msgs::Int32& msg)
{
    acc_pwm = msg.data;
    acc_msg_dT = 0;
}

//ROS Variables
ros::Subscriber<std_msgs::Int32> drum_sub("/drum_cmd", &drumCmdCB);
ros::Subscriber<std_msgs::Int32> tool_sub("/acc_cmd", &accCmdCB);
std_msgs::Float64MultiArray pub_arr;
ros::Publisher tool_raw_msg_pub("/tool_raw_info", &pub_arr);

void setup()
{
    // Initialize node
    nh.initNode();
    nh.advertise(tool_raw_msg_pub); // Initialize publisher
    nh.subscribe(drum_sub); // Initialize subscriber 1
    nh.subscribe(tool_sub); // Initialize subscriber 2

    //Setup Pins (With initial values)
    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);

    pinMode(DRUM_PWM_PIN, OUTPUT); analogWrite(DRUM_PWM_PIN, HIGH);
    pinMode(DRUM_ENC_A, INPUT_PULLUP);
    pinMode(DRUM_ENC_B, INPUT_PULLUP);
    pinMode(DRUM_DIR_PIN1, OUTPUT); digitalWrite(DRUM_DIR_PIN1, LOW);
    // pinMode(DRUM_DIR_PIN2, OUTPUT); digitalWrite(DRUM_DIR_PIN2, HIGH);

    //Linear Actuator
    pinMode(ACC_PWM_PIN, OUTPUT); analogWrite(DRUM_PWM_PIN, HIGH);
    pinMode(ACC_DIR_PIN1, OUTPUT); digitalWrite(ACC_DIR_PIN1, LOW);
    // pinMode(ACC_DIR_PIN2, OUTPUT); digitalWrite(ACC_DIR_PIN2, HIGH);
    pinMode(ACC_FEEDBACK_PIN, INPUT_PULLUP);

    //Current Sensors
    pinMode(CURR_SENS_DRUM, INPUT);
    pinMode(CURR_SENS_ACC, INPUT);

    //Interrupts to read encoder data
    attachInterrupt(digitalPinToInterrupt(DRUM_ENC_A), drum_position_interrupt, RISING);//Interrupt activates when a gets rising edge
    attachInterrupt(digitalPinToInterrupt(ACC_FEEDBACK_PIN), acc_position_interrupt, RISING);//Interrupt activates when pin gets rising edge

    // Store default values in _prev variables
    drum_read_prev = 0;
    acc_read_prev = 0;
    t_prev = millis();
    pub_arr.data = (float *)malloc(sizeof(float)*4);
    while(nh.connected()==false)
    {
      nh.spinOnce(); // Spin node 
      delay(50);
    }
    nh.loginfo("Setup Completed");
    node_started = true;
}
bool fwd_done = false;
void loop()
{
    //--------------------Command Drum-----------------------
    if (drum_msg_dT>3000) //if last message was within 3 seconds
    {
        drum_pwm = 0;
        // nh.loginfo("Locking Drum");
    }
    else 
    {
        // nh.loginfo("Commanding Drum "+ drum_pwm);
    }

    // if(fwd_done ==false && drum_ticks<6000)
    // {
    //     drum_pwm = 100;
    // }
    // else if(fwd_done == false && drum_ticks > 6000)
    // {
    //   fwd_done = true;
    // }
    // else if(drum_ticks>0)
    // {
    //   drum_pwm = -60;
    // }
    // else drum_pwm = 0;

    //write drum_pwm to pin
    if(drum_pwm>0)
    {
        digitalWrite(DRUM_DIR_PIN1, HIGH);
        // digitalWrite(DRUM_DIR_PIN2, LOW);
    }
    else
    {
        digitalWrite(DRUM_DIR_PIN1, LOW);
        // digitalWrite(DRUM_DIR_PIN2, HIGH);
    }
    analogWrite(DRUM_PWM_PIN, abs(drum_pwm));

    //--------------------Command Linear Actuator-----------------------
    if (acc_msg_dT>3000) //if last message was within 3 seconds
    {
      acc_pwm = 0;
      // nh.loginfo("Locking Linear Actuator");
    }
    else
    {
        // nh.loginfo("Commanding Linear Actuator "+ acc_pwm);
    }


    //write acc_pwm to pin
    if(acc_pwm>0)
    {
      digitalWrite(ACC_DIR_PIN1, HIGH);
      // digitalWrite(ACC_DIR_PIN2, LOW);
      acc_dir = 1;
    }
    else
    {
      digitalWrite(ACC_DIR_PIN1, LOW);
      // digitalWrite(ACC_DIR_PIN2, HIGH);
      acc_dir = 0;
    }
    analogWrite(ACC_PWM_PIN, abs(acc_pwm));

    // //--------------------Read Current Sensors-----------------------
    //Calculate dticks/dt for drum
    acc_read_curr=acc_ticks; drum_read_curr = drum_ticks;
    t_curr=millis();

    //Publish Feedback from Encoders and Current Sensors using tool_raw_cmd_pub
    pub_arr.data_length=4;
    // pub_arr.data[0] = (((double)drum_read_curr-(double)drum_read_prev)/((double)t_curr-(double)t_prev));
    pub_arr.data[0] = drum_ticks;
    pub_arr.data[1]= acc_ticks;
    pub_arr.data[2]= analogRead(CURR_SENS_DRUM);
    pub_arr.data[3]= analogRead(CURR_SENS_ACC);
    tool_raw_msg_pub.publish(&pub_arr);
    nh.loginfo( (String("Current Commands: ")+ String(acc_pwm)+ " "+String(drum_pwm)).c_str() );
    // nh.loginfo( (String("Current Readings: Drum Ticks")+ String(drum_ticks)+ " Acc ticks "+String(acc_ticks)).c_str() );

    //Update previous values
    drum_read_prev = drum_read_curr; acc_read_prev = acc_read_curr;
    t_prev = t_curr;
    nh.spinOnce(); // Spin node
    delay(100);
    drum_msg_dT+=100; 
    acc_msg_dT+=100;
}