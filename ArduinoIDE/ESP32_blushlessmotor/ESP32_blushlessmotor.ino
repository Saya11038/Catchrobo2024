#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <ESP32Servo.h>

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


#define LED_BUILTIN 2

#define LIMIT_SWITCH_R 23
#define LIMIT_SWITCH_THETA 22
#define LIMIT_SWITCH_Z 21

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#define ESC_Pin_1 12     //ESC 出力ピン
#define ESC_Pin_2 14
#define ESC_Pin_3 27

//ESC設定(μs)
#define ESC_Min_PWM 1000  //ESC最小PWM(μs)
#define ESC_Max_PWM 2000  //ESC最大PWM(μs)

Servo esc_1, esc_2, esc_3;


void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {

    msg.data = 0;

    if (digitalRead(LIMIT_SWITCH_R) == HIGH) {
      msg.data += 1;
    }
    if (digitalRead(LIMIT_SWITCH_THETA) == HIGH) {
      msg.data += 2;
    }
    if (digitalRead(LIMIT_SWITCH_Z) == HIGH) {
      msg.data += 4;
    }

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }

  int vol = 1300;
  esc_1.writeMicroseconds(vol);
  esc_2.writeMicroseconds(vol);
  esc_3.writeMicroseconds(vol);
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_BUILTIN, (msg->data == 0) ? LOW : HIGH);  
}


void setup() {
  set_microros_transports();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  
  
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

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_subscriber"));

  // create timer,
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // create executor
  // RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  msg.data = 0;

  pinMode(LIMIT_SWITCH_R, INPUT);
  pinMode(LIMIT_SWITCH_THETA, INPUT);
  pinMode(LIMIT_SWITCH_Z, INPUT);

  esc_1.attach(ESC_Pin_1);

  delay(100);
  esc_1.writeMicroseconds(ESC_Max_PWM);
  delay(2000);
  esc_1.writeMicroseconds(ESC_Min_PWM);
  delay(2000);

  esc_2.attach(ESC_Pin_2);

  delay(100);
  esc_2.writeMicroseconds(ESC_Max_PWM);
  delay(2000);
  esc_2.writeMicroseconds(ESC_Min_PWM);
  delay(2000);

  esc_3.attach(ESC_Pin_3);

  delay(100);
  esc_3.writeMicroseconds(ESC_Max_PWM);
  delay(2000);
  esc_3.writeMicroseconds(ESC_Min_PWM);
  delay(2000);
  
}


void loop() {
  delay(50);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}
