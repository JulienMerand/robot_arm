#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>

rcl_subscription_t subscriber_move;
// rcl_subscription_t subscriber_gripper;
// std_msgs__msg__Int32 msg_gripper;
std_msgs__msg__Int32MultiArray msg_move;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ##############################################################################################

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

// Definition des pins
#define ENA_6 22
#define DIR_6 23
#define PUL_6 24

#define ENA_5 27
#define DIR_5 28
#define PUL_5 29

#define ENA_4 30
#define DIR_4 31
#define PUL_4 32

#define ENA_3 34
#define DIR_3 35
#define PUL_3 36

#define ENA_1 38
#define DIR_1 39
#define PUL_1 40

#define ENA_2B 44
#define DIR_2B 45
#define PUL_2B 46

#define ENA_2A 49
#define DIR_2A 50
#define PUL_2A 51

#define PIN_GRIPPER 52

// Déclaration du gripper
Servo gripper;

// Déclaration des moteurs AccelStepper
AccelStepper stepper1(AccelStepper::DRIVER, PUL_1, DIR_1);
AccelStepper stepper2A(AccelStepper::DRIVER, PUL_2A, DIR_2A);
AccelStepper stepper2B(AccelStepper::DRIVER, PUL_2B, DIR_2B);
AccelStepper stepper3(AccelStepper::DRIVER, PUL_3, DIR_3);
AccelStepper stepper4(AccelStepper::DRIVER, PUL_4, DIR_4);
AccelStepper stepper5(AccelStepper::DRIVER, PUL_5, DIR_5);
AccelStepper stepper6(AccelStepper::DRIVER, PUL_6, DIR_6);

AccelStepper stepper[] = {stepper1, stepper2A, stepper2B, stepper3, stepper4, stepper5, stepper6};
MultiStepper steppers;

long step[7];

// Paramètres
#define speed 500
#define accel 100

// ##############################################################################################

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void callback_move(const void * msgin)
{  
  const std_msgs__msg__Int32MultiArray * msg_move = (const std_msgs__msg__Int32MultiArray *)msgin;  
  
  for(int i=0; i<msg_move->data.size; i++){
    step[i] = msg_move->data.data[i];
  }

  // steppers.moveTo(step);
  // steppers.runSpeedToPosition();
  // delay(100);

}

// void callback_gripper(const void * msgin)
// {  
//   const std_msgs__msg__Int32 * msg_gripper = (const std_msgs__msg__Int32 *)msgin;
//   msg_gripper->data == 1 ? open_gripper() : close_gripper(); 
// }

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

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

  for(int i=0; i<7; i++){
    int v = 700;
    stepper[i].setMaxSpeed(v);
    stepper[i].setAcceleration(accel);
  }

  //Gripper
  gripper.attach(PIN_GRIPPER);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber_move,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "micro_ros_arduino_subscriber_move"));
  
  // // create subscriber
  // RCCHECK(rclc_subscription_init_default(
  //   &subscriber_gripper,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //   "micro_ros_arduino_subscriber_gripper"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_move, &msg_move, &callback_move, ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_gripper, &msg_gripper, &callback_gripper, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}