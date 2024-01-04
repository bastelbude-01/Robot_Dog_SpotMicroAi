#include <micro_ros_arduino.h>
#include <ESP32Servo.h>

#include <thread>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>  // Twist msg für cmd_vel
#include <std_msgs/msg/int8.h>        // Int8 msg für zahlen von -128 bis 127

rcl_subscription_t rgb_subscriber;    // Led Subscriber
geometry_msgs__msg__Twist msg;

rcl_subscription_t servo_sub;         // Blower Cases
std_msgs__msg__Int8 servo_msg;

rcl_subscription_t fr_speed_sub;         // Fräse Speed
std_msgs__msg__Int8 speed_msg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;


#define RH_RGB_ROT_PIN 19             // RGB Pins
#define RH_RGB_GRUEN_PIN 18
#define RH_RGB_BLAU_PIN 5

#define LH_RGB_ROT_PIN 0
#define LH_RGB_GRUEN_PIN 2
#define LH_RGB_BLAU_PIN 15

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}


Servo fr_;
Servo we_;
Servo bl_;
Servo fe_;
Servo free_;

int fr_Pin = 17; // Fräse
int we_Pin = 16; // Welle
int bl_Pin = 4; // Blower
int fe_Pin = 22; // fecher
int free_Pin = 23; // Freier Pin

int pos = 0;
int pos_fr_ = 15;
int pos_fr_off_ = 0;
int pos_we_ = 90;
int pos_bl_ = 0;
int pos_fe_ = 0;
int pos_free_ = 0;

bool fecher_running = false;


void error_loop() {
  while (1) {
    digitalWrite(RH_RGB_BLAU_PIN, !digitalRead(RH_RGB_BLAU_PIN));
    digitalWrite(LH_RGB_BLAU_PIN, !digitalRead(LH_RGB_BLAU_PIN));
    delay(100);
  }
}

int limitToMaxValue(int value, int maxLimit)
{
  if (value > maxLimit) {
    return maxLimit;
  }
  else
  {
    return value;
  }
}
void fecher()
{
  fecher_running = true;
  unsigned long start_time = millis();

  while (millis()-start_time <=50000) {
    for (pos = 0; pos <= 180; pos += 1) {
      fe_.write(pos);
      delay(15);
    }
    for (pos = 180; pos >= 0; pos -= 1) {
      fe_.write(pos);
      delay(15);
    }
  }
}
void fraes_speed_callback(const void *msgin) {
  const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;
  pos_fr_ = msg->data;
}


void servo_callback(const void *msgin) {
  const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;
  int8_t value = msg->data;

  switch (value) {
    case 0: // All off
      fr_.write(pos_fr_);
      we_.write(pos_we_);
      bl_.write(pos_bl_);
      fe_.write(pos_fe_);
      fecher_running = false;

      break;
    case 1: // All on
      fr_.write(pos_fr_);
      we_.write(15);
      bl_.write(25);
      if(!fecher_running){
        std::thread(fecher).detach();
        }
      break;
    case 2: // only full trailer
      we_.write(15);
      bl_.write(25);
      if(!fecher_running){
        std::thread(fecher).detach();
        }
      break;
    case 3: // only snowfräse
      fr_.write(pos_fr_);
      fecher_running = false;
      break;
    default:
      break;
  }

}

void rgb(int value)
{
  switch (value) {
    case 0: // Alle Aus
      digitalWrite(RH_RGB_ROT_PIN, LOW);
      digitalWrite(RH_RGB_GRUEN_PIN, LOW);
      digitalWrite(RH_RGB_BLAU_PIN, LOW);

      digitalWrite(LH_RGB_ROT_PIN, LOW);
      digitalWrite(LH_RGB_GRUEN_PIN, LOW);
      digitalWrite(LH_RGB_BLAU_PIN, LOW);
      break;
    case 1: // Rot Rechts
      digitalWrite(RH_RGB_ROT_PIN, HIGH);
      digitalWrite(RH_RGB_GRUEN_PIN, LOW);
      digitalWrite(RH_RGB_BLAU_PIN, LOW);

      digitalWrite(LH_RGB_ROT_PIN, LOW);
      digitalWrite(LH_RGB_GRUEN_PIN, LOW);
      digitalWrite(LH_RGB_BLAU_PIN, LOW);
      break;
    case 2:// Rot Links
      digitalWrite(RH_RGB_ROT_PIN, LOW);
      digitalWrite(RH_RGB_GRUEN_PIN, LOW);
      digitalWrite(RH_RGB_BLAU_PIN, LOW);

      digitalWrite(LH_RGB_ROT_PIN, HIGH);
      digitalWrite(LH_RGB_GRUEN_PIN, LOW);
      digitalWrite(LH_RGB_BLAU_PIN, LOW);
      break;
    case 3: // Alle Rot
      digitalWrite(RH_RGB_ROT_PIN, HIGH);
      digitalWrite(RH_RGB_GRUEN_PIN, LOW);
      digitalWrite(RH_RGB_BLAU_PIN, LOW);

      digitalWrite(LH_RGB_ROT_PIN, HIGH);
      digitalWrite(LH_RGB_GRUEN_PIN, LOW);
      digitalWrite(LH_RGB_BLAU_PIN, LOW);
      break;
    case 4: // Gruen Rechts
      digitalWrite(RH_RGB_ROT_PIN, LOW);
      digitalWrite(RH_RGB_GRUEN_PIN, HIGH);
      digitalWrite(RH_RGB_BLAU_PIN, LOW);

      digitalWrite(LH_RGB_ROT_PIN, LOW);
      digitalWrite(LH_RGB_GRUEN_PIN, LOW);
      digitalWrite(LH_RGB_BLAU_PIN, LOW);
      break;
    case 5: // Gruen Links
      digitalWrite(RH_RGB_ROT_PIN, LOW);
      digitalWrite(RH_RGB_GRUEN_PIN, LOW);
      digitalWrite(RH_RGB_BLAU_PIN, LOW);

      digitalWrite(LH_RGB_ROT_PIN, LOW);
      digitalWrite(LH_RGB_GRUEN_PIN, HIGH);
      digitalWrite(LH_RGB_BLAU_PIN, LOW);
      break;
    case 6: // Alle Gruen
      digitalWrite(RH_RGB_ROT_PIN, LOW);
      digitalWrite(RH_RGB_GRUEN_PIN, HIGH);
      digitalWrite(RH_RGB_BLAU_PIN, LOW);

      digitalWrite(LH_RGB_ROT_PIN, LOW);
      digitalWrite(LH_RGB_GRUEN_PIN, HIGH);
      digitalWrite(LH_RGB_BLAU_PIN, LOW);
      break;
    case 7: // Blau Rechts
      digitalWrite(RH_RGB_ROT_PIN, LOW);
      digitalWrite(RH_RGB_GRUEN_PIN, LOW);
      digitalWrite(RH_RGB_BLAU_PIN, HIGH);

      digitalWrite(LH_RGB_ROT_PIN, LOW);
      digitalWrite(LH_RGB_GRUEN_PIN, LOW);
      digitalWrite(LH_RGB_BLAU_PIN, LOW);
      break;
    case 8: // Blau Links
      digitalWrite(RH_RGB_ROT_PIN, LOW);
      digitalWrite(RH_RGB_GRUEN_PIN, LOW);
      digitalWrite(RH_RGB_BLAU_PIN, LOW);

      digitalWrite(LH_RGB_ROT_PIN, LOW);
      digitalWrite(LH_RGB_GRUEN_PIN, LOW);
      digitalWrite(LH_RGB_BLAU_PIN, HIGH);
      break;
    case 9: // Alle Blau
      digitalWrite(RH_RGB_ROT_PIN, LOW);
      digitalWrite(RH_RGB_GRUEN_PIN, LOW);
      digitalWrite(RH_RGB_BLAU_PIN, HIGH);

      digitalWrite(LH_RGB_ROT_PIN, LOW);
      digitalWrite(LH_RGB_GRUEN_PIN, LOW);
      digitalWrite(LH_RGB_BLAU_PIN, HIGH);
      break;
    case 10: // Gelb Rechts
      digitalWrite(RH_RGB_ROT_PIN, HIGH);
      digitalWrite(RH_RGB_GRUEN_PIN, HIGH);
      digitalWrite(RH_RGB_BLAU_PIN, LOW);

      digitalWrite(LH_RGB_ROT_PIN, LOW);
      digitalWrite(LH_RGB_GRUEN_PIN, LOW);
      digitalWrite(LH_RGB_BLAU_PIN, LOW);
      break;
    case 11: // Gelb Links
      digitalWrite(RH_RGB_ROT_PIN, LOW);
      digitalWrite(RH_RGB_GRUEN_PIN, LOW);
      digitalWrite(RH_RGB_BLAU_PIN, LOW);

      digitalWrite(LH_RGB_ROT_PIN, HIGH);
      digitalWrite(LH_RGB_GRUEN_PIN, HIGH);
      digitalWrite(LH_RGB_BLAU_PIN, LOW);
      break;
    case 12: // Alle Gelb
      digitalWrite(RH_RGB_ROT_PIN, HIGH);
      digitalWrite(RH_RGB_GRUEN_PIN, HIGH);
      digitalWrite(RH_RGB_BLAU_PIN, LOW);

      digitalWrite(LH_RGB_ROT_PIN, HIGH);
      digitalWrite(LH_RGB_GRUEN_PIN, HIGH);
      digitalWrite(LH_RGB_BLAU_PIN, LOW);
      break;
    case 13: // Alle Weiss
      digitalWrite(RH_RGB_ROT_PIN, HIGH);
      digitalWrite(RH_RGB_GRUEN_PIN, HIGH);
      digitalWrite(RH_RGB_BLAU_PIN, HIGH);

      digitalWrite(LH_RGB_BLAU_PIN, HIGH);
      digitalWrite(LH_RGB_GRUEN_PIN, HIGH);
      digitalWrite(LH_RGB_BLAU_PIN, HIGH);
      break;
    default:
      break;
  }

}


//twist message cb
void rgb_cmdvel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  int8_t value = 0;

  if (msg->angular.z == 0.0 )
  {
    value = 0;
    rgb(value);
  }

  else if (msg->angular.z <= 0.2)
  {
    value = 10;
    rgb(value);
  }
  else if (msg->angular.z >= -0.2)
  {
    value = 11;
    rgb(value);
  }

}

void setup() {
  // set_microros_transports();
  set_microros_wifi_transports("FRITZ!Box 7490", "54908635459129454475", "192.168.178.86", 8868);
  pinMode(RH_RGB_ROT_PIN, OUTPUT);
  pinMode(RH_RGB_GRUEN_PIN, OUTPUT);
  pinMode(RH_RGB_BLAU_PIN, OUTPUT);

  pinMode(LH_RGB_ROT_PIN, OUTPUT);
  pinMode(LH_RGB_GRUEN_PIN, OUTPUT);
  pinMode(LH_RGB_BLAU_PIN, OUTPUT);


  digitalWrite(RH_RGB_ROT_PIN, LOW);
  digitalWrite(RH_RGB_GRUEN_PIN, LOW);
  digitalWrite(RH_RGB_BLAU_PIN, LOW);

  digitalWrite(LH_RGB_ROT_PIN, LOW);
  digitalWrite(LH_RGB_GRUEN_PIN, LOW);
  digitalWrite(LH_RGB_BLAU_PIN, LOW);

  fecher_running = false;


  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  // standard 50 hz servo
  fr_.setPeriodHertz(50);
  we_.setPeriodHertz(50);
  bl_.setPeriodHertz(50);
  fe_.setPeriodHertz(50);
  free_.setPeriodHertz(50);

  fr_.attach(fr_Pin, 1000, 2000);
  we_.attach(we_Pin, 1000, 2000);
  bl_.attach(bl_Pin, 1000, 2000);
  fe_.attach(fe_Pin, 1000, 2000);
  free_.attach(free_Pin, 1000, 2000);

  delay(3000);

  fr_.write(pos_fr_off_);
  bl_.write(pos_bl_);



  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "trailer_snowblower", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
            &rgb_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "leo_driver/cmd_vel_unstamped"));

  RCCHECK(rclc_subscription_init_default(
            &servo_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
            "salt"));

  RCCHECK(rclc_subscription_init_default(
            &fr_speed_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
            "blower_speed"));  // fraes_speed_callback

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &rgb_subscriber, &msg, &rgb_cmdvel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &fr_speed_sub, &speed_msg, &fraes_speed_callback, ON_NEW_DATA));

}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
