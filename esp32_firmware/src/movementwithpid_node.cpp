#include <Arduino.h>
#include <cmath>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

#include "../config/move.h"

// -------- Helpers --------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) init = uxr_millis(); \
  if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

// -------- Config --------
#define CMD_TIMEOUT_MS 500
#define PWM_STEPS 20

// -------- ROS --------
rcl_publisher_t distance_publisher;
std_msgs__msg__Float32 distance_msg;

rcl_subscription_t motor_subscriber;
geometry_msgs__msg__Twist motor_msg;

rcl_subscription_t reset_subscriber;
std_msgs__msg__Bool reset_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

// -------- Encoder --------
volatile int32_t motor1_encoder_ticks = 0;
volatile int32_t motor2_encoder_ticks = 0;
volatile int32_t motor3_encoder_ticks = 0;
volatile int32_t motor4_encoder_ticks = 0;

float distance_inside_planter = 0.0;
const float WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;

unsigned long prev_cmd_time = 0;

// -------- Encoder ISR --------
void IRAM_ATTR handleEncoder_MOTOR1_A(){ motor1_encoder_ticks += digitalRead(MOTOR1_ENCODER_PIN_B) ? 1 : -1; }
void IRAM_ATTR handleEncoder_MOTOR2_A(){ motor2_encoder_ticks += digitalRead(MOTOR2_ENCODER_PIN_B) ? 1 : -1; }
void IRAM_ATTR handleEncoder_MOTOR3_A(){ motor3_encoder_ticks += digitalRead(MOTOR3_ENCODER_PIN_B) ? 1 : -1; }
void IRAM_ATTR handleEncoder_MOTOR4_A(){ motor4_encoder_ticks += digitalRead(MOTOR4_ENCODER_PIN_B) ? 1 : -1; }

void IRAM_ATTR handleEncoder_MOTOR1_B(){ motor1_encoder_ticks += digitalRead(MOTOR1_ENCODER_PIN_A) ? -1 : 1; }
void IRAM_ATTR handleEncoder_MOTOR2_B(){ motor2_encoder_ticks += digitalRead(MOTOR2_ENCODER_PIN_A) ? -1 : 1; }
void IRAM_ATTR handleEncoder_MOTOR3_B(){ motor3_encoder_ticks += digitalRead(MOTOR3_ENCODER_PIN_A) ? -1 : 1; }
void IRAM_ATTR handleEncoder_MOTOR4_B(){ motor4_encoder_ticks += digitalRead(MOTOR4_ENCODER_PIN_A) ? -1 : 1; }

// -------- Forward declarations --------
void rclErrorLoop();
bool createEntities();
bool destroyEntities();
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);

void driveDifferential();
void setMotorPWM(int a, int b, float rpm);
void Encoder();
void publishData();

// =================================================
// SETUP
// =================================================

void setup()
{
  Serial.begin(115200);
  delay(2000);

  set_microros_serial_transports(Serial);

  pinMode(MOTOR1_IN_A, OUTPUT);
  pinMode(MOTOR1_IN_B, OUTPUT);
  pinMode(MOTOR2_IN_A, OUTPUT);
  pinMode(MOTOR2_IN_B, OUTPUT);
  pinMode(MOTOR3_IN_A, OUTPUT);
  pinMode(MOTOR3_IN_B, OUTPUT);
  pinMode(MOTOR4_IN_A, OUTPUT);
  pinMode(MOTOR4_IN_B, OUTPUT);

  pinMode(MOTOR1_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(MOTOR1_ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(MOTOR2_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(MOTOR2_ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(MOTOR3_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(MOTOR3_ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(MOTOR4_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(MOTOR4_ENCODER_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_PIN_A), handleEncoder_MOTOR1_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_PIN_B), handleEncoder_MOTOR1_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCODER_PIN_A), handleEncoder_MOTOR2_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCODER_PIN_B), handleEncoder_MOTOR2_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_ENCODER_PIN_A), handleEncoder_MOTOR3_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_ENCODER_PIN_B), handleEncoder_MOTOR3_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR4_ENCODER_PIN_A), handleEncoder_MOTOR4_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR4_ENCODER_PIN_B), handleEncoder_MOTOR4_B, CHANGE);

  state = WAITING_AGENT;
}

// =================================================
// LOOP
// =================================================

void loop()
{
  switch (state)
  {
    case WAITING_AGENT:

      EXECUTE_EVERY_N_MS(1500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100,10))
        ? AGENT_AVAILABLE
        : WAITING_AGENT;
      );

      break;

    case AGENT_AVAILABLE:

      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;

      if(state == WAITING_AGENT)
        destroyEntities();

      break;

    case AGENT_CONNECTED:

      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100,10))
        ? AGENT_CONNECTED
        : AGENT_DISCONNECTED;
      );

      if(state == AGENT_CONNECTED)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

      break;

    case AGENT_DISCONNECTED:

      destroyEntities();
      state = WAITING_AGENT;

      break;
  }
}

// =================================================
// ROS CALLBACK
// =================================================

void twistCallback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  motor_msg = *msg;

  prev_cmd_time = millis();
}

void resetCallback(const void *msgin)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;

  if(msg->data)
  {
    distance_inside_planter = 0;

    motor1_encoder_ticks = 0;
    motor2_encoder_ticks = 0;
    motor3_encoder_ticks = 0;
    motor4_encoder_ticks = 0;
  }
}

// =================================================
// CONTROL LOOP
// =================================================

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if(timer != NULL)
  {
    Encoder();
    driveDifferential();
    publishData();
  }
}

// =================================================
// DRIVE DIFFERENTIAL
// =================================================

void driveDifferential()
{
  if(millis() - prev_cmd_time > CMD_TIMEOUT_MS)
  {
    digitalWrite(MOTOR1_IN_A, LOW);
    digitalWrite(MOTOR1_IN_B, LOW);
    digitalWrite(MOTOR2_IN_A, LOW);
    digitalWrite(MOTOR2_IN_B, LOW);
    digitalWrite(MOTOR3_IN_A, LOW);
    digitalWrite(MOTOR3_IN_B, LOW);
    digitalWrite(MOTOR4_IN_A, LOW);
    digitalWrite(MOTOR4_IN_B, LOW);
    return;
  }

  float Vx = motor_msg.linear.x;
  float Wz = motor_msg.angular.z;

  float wheel_left  = Vx - (Wz * LR_WHEELS_DISTANCE * 0.5);
  float wheel_right = Vx + (Wz * LR_WHEELS_DISTANCE * 0.5);

  float left_rpm  = wheel_left;
  float right_rpm = wheel_right;

  setMotorPWM(MOTOR1_IN_A, MOTOR1_IN_B, left_rpm);
  setMotorPWM(MOTOR3_IN_A, MOTOR3_IN_B, left_rpm);

  // ⭐ IMPORTANT FIX
  setMotorPWM(MOTOR2_IN_A, MOTOR2_IN_B, -right_rpm);
  setMotorPWM(MOTOR4_IN_A, MOTOR4_IN_B, -right_rpm);
}

// =================================================
// MOTOR DRIVER
// =================================================

void setMotorPWM(int a, int b, float rpm)
{
  if(rpm > 0)
  {
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
  }
  else if(rpm < 0)
  {
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
  }
  else
  {
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);
  }
}

// =================================================
// ENCODER
// =================================================

void Encoder()
{
  static int32_t prev1=0,prev2=0,prev3=0,prev4=0;

  int32_t t1 = motor1_encoder_ticks;
  int32_t t2 = motor2_encoder_ticks;
  int32_t t3 = motor3_encoder_ticks;
  int32_t t4 = motor4_encoder_ticks;

  int32_t d1 = t1-prev1;
  int32_t d2 = t2-prev2;
  int32_t d3 = t3-prev3;
  int32_t d4 = t4-prev4;

  prev1=t1;
  prev2=t2;
  prev3=t3;
  prev4=t4;

  float avg_ticks=(d1+d2+d3+d4)/4.0;
  float rev=avg_ticks/ENCODER_TICKS_PER_REV;

  distance_inside_planter -= rev*WHEEL_CIRCUMFERENCE;
}

// =================================================
// PUBLISH
// =================================================

void publishData()
{
  distance_msg.data = distance_inside_planter;

  RCSOFTCHECK(rcl_publish(&distance_publisher,&distance_msg,NULL));
}

// =================================================
// CREATE ENTITIES
// =================================================

bool createEntities()
{
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();

  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options,77));

  RCCHECK(rclc_support_init_with_options(&support,0,NULL,&init_options,&allocator));

  RCCHECK(rclc_node_init_default(&node,"quin_robot_node","",&support));

  RCCHECK(rclc_publisher_init_best_effort(
    &distance_publisher,&node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32),
    "/quin/distance_inside_planter"));

  RCCHECK(rclc_subscription_init_best_effort(
    &motor_subscriber,&node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),
    "/quin/move_command"));

  RCCHECK(rclc_subscription_init_best_effort(
    &reset_subscriber,&node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Bool),
    "/quin/reset_distance"));

  RCCHECK(rclc_timer_init_default(
    &control_timer,&support,
    RCL_MS_TO_NS(20),
    controlCallback));

  executor = rclc_executor_get_zero_initialized_executor();

  RCCHECK(rclc_executor_init(&executor,&support.context,3,&allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor,&motor_subscriber,&motor_msg,&twistCallback,ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor,&reset_subscriber,&reset_msg,&resetCallback,ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor,&control_timer));

  return true;
}

// =================================================
// DESTROY
// =================================================

bool destroyEntities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context,0);

  RCSOFTCHECK(rcl_publisher_fini(&distance_publisher,&node));
  RCSOFTCHECK(rcl_subscription_fini(&motor_subscriber,&node));
  RCSOFTCHECK(rcl_subscription_fini(&reset_subscriber,&node));
  RCSOFTCHECK(rcl_timer_fini(&control_timer));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rclc_support_fini(&support));

  return true;
}

// =================================================
// ERROR
// =================================================

void rclErrorLoop()
{
  while(true)
  {
    delay(100);
  }
}