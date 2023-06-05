// Run on terminal : 
// MicroXRCEAgent serial --dev /dev/ttyACM0 -b 115200

#include "define.h"

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg_pub;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

bool start = true;
bool cb_called = false;
bool gripper_moveable = false;
int gripper_value;
long step[7];
int speed = 50;
int accel = 100;


#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{ 
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  
  for(int i=0; i<7; i++){
    step[i] = long(msg->data.data[i]);
  }

  if(msg->data.data[7] > 75 && msg->data.data[7] < 190){
      gripper_value = msg->data.data[7];
      gripper_moveable = true;
    }

  speed = msg->data.data[8];
  accel = msg->data.data[9];

  cb_called = true;
  start = false;
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  

  // Broche de contrôle d'activation du moteur
  stepper[0].setEnablePin(ENA_1);
  stepper[1].setEnablePin(ENA_2A);
  stepper[2].setEnablePin(ENA_2B);
  stepper[3].setEnablePin(ENA_3);
  stepper[4].setEnablePin(ENA_4);
  stepper[5].setEnablePin(ENA_5);
  stepper[6].setEnablePin(ENA_6);

  // Activation des moteurs
  for(int i=0; i<7; i++){
    stepper[i].disableOutputs();
  }

  for(int i=0; i<7; i++){
    steppers.addStepper(stepper[i]);
  }

  stepper[0].setMaxSpeed(speed);
  stepper[0].setAcceleration(accel);
  stepper[1].setMaxSpeed(speed);
  stepper[1].setAcceleration(accel);
  stepper[2].setMaxSpeed(speed);
  stepper[2].setAcceleration(accel);
  stepper[3].setMaxSpeed(speed*5);
  stepper[3].setAcceleration(accel*5);
  stepper[4].setMaxSpeed(speed);
  stepper[4].setAcceleration(accel);
  stepper[5].setMaxSpeed(speed);
  stepper[5].setAcceleration(accel);
  stepper[6].setMaxSpeed(speed);
  stepper[6].setAcceleration(accel);

  gripper.attach(PIN_GRIPPER);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "micro_ros_arduino_subscriber"));
  
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));

  // // create timer,
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg_pub.data = 0;

  // Memory 
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_string_capacity = 50;
  conf.max_ros2_type_sequence_capacity = 10;
  conf.max_basic_type_sequence_capacity = 10;

  bool success = micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    &msg,
    conf
  );

  
}

void loop() {

  if(cb_called){

    stepper[0].setMaxSpeed(speed*10.25);
    stepper[0].setAcceleration(accel*10.25);
    stepper[1].setMaxSpeed(speed*5.45);
    stepper[1].setAcceleration(accel*5.45);
    stepper[2].setMaxSpeed(speed*5.45);
    stepper[2].setAcceleration(accel*5.45);
    stepper[3].setMaxSpeed(speed*22.75);
    stepper[3].setAcceleration(accel*22.75);
    stepper[4].setMaxSpeed(speed);
    stepper[4].setAcceleration(accel);
    stepper[5].setMaxSpeed(speed*4.5);
    stepper[5].setAcceleration(accel*4.5);
    stepper[6].setMaxSpeed(speed);
    stepper[6].setAcceleration(accel);

    steppers.moveTo(step);
    bool ret = steppers.runSpeedToPosition();

    if(gripper_moveable){
      gripper.write(gripper_value);
    }
    //if(ret){
    msg_pub.data = 1;
    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
    //}
    
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    msg_pub.data = 0;
    cb_called = false;
    gripper_moveable = false;

  }
  else{
    msg_pub.data = 0;
    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
  }

  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // delay(10);
}