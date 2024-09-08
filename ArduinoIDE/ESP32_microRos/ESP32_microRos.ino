#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <ESP32Servo.h>


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 sensor;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#define LED_BUILTIN 2

#define LIMIT_SWITCH_R 23
#define LIMIT_SWITCH_THETA 22
#define LIMIT_SWITCH_Z 21

#define ESC_Pin_1 12     //ESC 出力ピン
#define ESC_Pin_2 14
#define ESC_Pin_3 27

//ESC設定(μs)
#define ESC_Min_PWM 1000  //ESC最小PWM(μs)
#define ESC_Max_PWM 2000  //ESC最大PWM(μs)


Servo esc_1, esc_2, esc_3;
int vol = 1300;
int esc_a, esc_b, esc_c;


void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Callback
void callback(const void *msgin){

  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  esc_a = (msg->data >> 0) & 1;
  esc_b = (msg->data >> 1) & 1;
  esc_c = (msg->data >> 2) & 1;

  if (esc_a == 1) {
    esc_1.writeMicroseconds(vol);
  } else {
    esc_1.writeMicroseconds(ESC_Min_PWM);
  }
  if (esc_b == 1) {
    esc_2.writeMicroseconds(vol);
  } else {
    esc_2.writeMicroseconds(ESC_Min_PWM);
  }
  if (esc_c == 1) {
    esc_3.writeMicroseconds(vol);
  } else {
    esc_3.writeMicroseconds(ESC_Min_PWM);
  }

  sensor.data = 0;

  if (digitalRead(LIMIT_SWITCH_R) == HIGH) {
    sensor.data += 1;
  }
  if (digitalRead(LIMIT_SWITCH_THETA) == HIGH) {
    sensor.data += 2;
  }
  if (digitalRead(LIMIT_SWITCH_Z) == HIGH) {
    sensor.data += 4;
  }
	// 配信
	RCSOFTCHECK(rcl_publish(&publisher, &sensor, NULL));
}

void setup(){
	// 通信の初期化
	// USB経由の場合
	set_microros_transports();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  

	// 初期化完了までの待機時間
	delay(2000);
	
	// micro-ROSのためのメモリ管理
	allocator = rcl_get_default_allocator();

	// micro-ROSのためのサポートクラス
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// ノードの生成
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

	// Publisher 作成
	RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));

	// Subscriber 作成
	RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_subscriber"));
	
	// コールバックを管理ためのexecutor
	// Subscriber、Timer、Serviceなどもコールバック関数を設定する
	// Publisherだけなら、以降の処理は必要ない
	
	int callback_size = 1;	// コールバックを行う数
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, callback_size, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &callback, ON_NEW_DATA));

  msg.data = 0;
  sensor.data = 0;

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

void loop(){
	// 受信
  	RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(200)));
  	delay(10);
}
