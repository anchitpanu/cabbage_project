#include <Arduino.h>
#include <cmath>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

#include "../config/move.h"

// ================= CONFIG =================

#define CMD_TIMEOUT_MS 500
#define CONTROL_PERIOD 20   // ms
#define TICK_THRESHOLD 3    // ticks น้อยกว่านี้ถือว่า noise → ไม่นับระยะ

const float MAX_LINEAR_SPEED = (MOTOR_MAX_RPM / 60.0f) * (M_PI * WHEEL_DIAMETER);

// ADJUST VALUE HERE
float KP_STRAIGHT = 0.015;
float KI_STRAIGHT = 0.0002;
float KD_STRAIGHT = 0.001;

float ENCODER_FILTER = 0.7;
const float INTEGRAL_LIMIT = 30.0;

#define PWM_FREQ    20000
#define PWM_RES     8

#define PWM_CH_M1_A 0
#define PWM_CH_M1_B 1
#define PWM_CH_M2_A 2
#define PWM_CH_M2_B 3
#define PWM_CH_M3_A 4
#define PWM_CH_M3_B 5
#define PWM_CH_M4_A 6
#define PWM_CH_M4_B 7

// ================= ROS =================

rcl_publisher_t distance_publisher;
std_msgs__msg__Float32 distance_msg;

rcl_publisher_t debug_dl_publisher;
rcl_publisher_t debug_dr_publisher;
rcl_publisher_t debug_err_publisher;
rcl_publisher_t debug_corr_publisher;
std_msgs__msg__Float32 debug_dl_msg;
std_msgs__msg__Float32 debug_dr_msg;
std_msgs__msg__Float32 debug_err_msg;
std_msgs__msg__Float32 debug_corr_msg;

rcl_publisher_t robot_ready_publisher;
std_msgs__msg__Bool robot_ready_msg;

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

// ================= ENCODER =================

volatile int32_t motor1_encoder_ticks = 0;
volatile int32_t motor2_encoder_ticks = 0;
volatile int32_t motor3_encoder_ticks = 0;
volatile int32_t motor4_encoder_ticks = 0;

int32_t prev_left_ticks  = 0;
int32_t prev_right_ticks = 0;

float filtered_error = 0;
float prev_error     = 0;
float integral_error = 0;

float distance_inside_planter = 0.0;
const float WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;

unsigned long prev_cmd_time = 0;
bool motors_stopped = true;  // track motor state เพื่อหยุดนับระยะตอนหยุด

// ================= ENCODER ISR =================

void IRAM_ATTR handleEncoder_MOTOR1_A()
{
  int dir = digitalRead(MOTOR1_ENCODER_PIN_B) ? 1 : -1;
  motor1_encoder_ticks += MOTOR1_ENCODER_INV ? -dir : dir;
}
void IRAM_ATTR handleEncoder_MOTOR1_B()
{
  int dir = digitalRead(MOTOR1_ENCODER_PIN_A) ? -1 : 1;
  motor1_encoder_ticks += MOTOR1_ENCODER_INV ? -dir : dir;
}

void IRAM_ATTR handleEncoder_MOTOR2_A()
{
  int dir = digitalRead(MOTOR2_ENCODER_PIN_B) ? 1 : -1;
  motor2_encoder_ticks += MOTOR2_ENCODER_INV ? -dir : dir;
}
void IRAM_ATTR handleEncoder_MOTOR2_B()
{
  int dir = digitalRead(MOTOR2_ENCODER_PIN_A) ? -1 : 1;
  motor2_encoder_ticks += MOTOR2_ENCODER_INV ? -dir : dir;
}

void IRAM_ATTR handleEncoder_MOTOR3_A()
{
  int dir = digitalRead(MOTOR3_ENCODER_PIN_B) ? 1 : -1;
  motor3_encoder_ticks += MOTOR3_ENCODER_INV ? -dir : dir;
}
void IRAM_ATTR handleEncoder_MOTOR3_B()
{
  int dir = digitalRead(MOTOR3_ENCODER_PIN_A) ? -1 : 1;
  motor3_encoder_ticks += MOTOR3_ENCODER_INV ? -dir : dir;
}

void IRAM_ATTR handleEncoder_MOTOR4_A()
{
  int dir = digitalRead(MOTOR4_ENCODER_PIN_B) ? 1 : -1;
  motor4_encoder_ticks += MOTOR4_ENCODER_INV ? -dir : dir;
}
void IRAM_ATTR handleEncoder_MOTOR4_B()
{
  int dir = digitalRead(MOTOR4_ENCODER_PIN_A) ? -1 : 1;
  motor4_encoder_ticks += MOTOR4_ENCODER_INV ? -dir : dir;
}

// ================= Forward declarations =================

bool createEntities();
bool destroyEntities();
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void driveDifferential();
void setMotorPWM(int ch_a, int ch_b, bool invert, float speed);
void Encoder();
void publishData();
void twistCallback(const void *msgin);
void resetCallback(const void *msgin);

// ================= SETUP =================

void setup()
{
  Serial.begin(115200);
  delay(2000);

  set_microros_serial_transports(Serial);

  ledcSetup(PWM_CH_M1_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_M1_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR1_IN_A, PWM_CH_M1_A);
  ledcAttachPin(MOTOR1_IN_B, PWM_CH_M1_B);

  ledcSetup(PWM_CH_M2_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_M2_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR2_IN_A, PWM_CH_M2_A);
  ledcAttachPin(MOTOR2_IN_B, PWM_CH_M2_B);

  ledcSetup(PWM_CH_M3_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_M3_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR3_IN_A, PWM_CH_M3_A);
  ledcAttachPin(MOTOR3_IN_B, PWM_CH_M3_B);

  ledcSetup(PWM_CH_M4_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_M4_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR4_IN_A, PWM_CH_M4_A);
  ledcAttachPin(MOTOR4_IN_B, PWM_CH_M4_B);

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

// ================= LOOP =================

void loop()
{
  switch (state)
  {
    case WAITING_AGENT:
      if (RMW_RET_OK == rmw_uros_ping_agent(100, 10))
        state = AGENT_AVAILABLE;
      break;
    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      break;
    case AGENT_CONNECTED:
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      if (RMW_RET_OK != rmw_uros_ping_agent(100, 10))
        state = AGENT_DISCONNECTED;
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }
}

// ================= CALLBACKS =================

void twistCallback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  motor_msg = *msg;
  prev_cmd_time = millis();
}

void resetCallback(const void *msgin)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data)
  {
    distance_inside_planter = 0;
    motor1_encoder_ticks = 0;
    motor2_encoder_ticks = 0;
    motor3_encoder_ticks = 0;
    motor4_encoder_ticks = 0;
    filtered_error = 0;
    prev_error     = 0;
    integral_error = 0;
    prev_left_ticks  = 0;
    prev_right_ticks = 0;

    robot_ready_msg.data = true;
    rcl_publish(&robot_ready_publisher , &robot_ready_msg, NULL);
  }
}

// ================= CONTROL LOOP =================

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL)
  {
    driveDifferential();
    Encoder();
    publishData();
  }
}

// ================= DRIVE =================
void driveDifferential()
{
  if (millis() - prev_cmd_time > CMD_TIMEOUT_MS)
  {
    setMotorPWM(PWM_CH_M1_A, PWM_CH_M1_B, MOTOR1_INV, 0);
    setMotorPWM(PWM_CH_M2_A, PWM_CH_M2_B, MOTOR2_INV, 0);
    setMotorPWM(PWM_CH_M3_A, PWM_CH_M3_B, MOTOR3_INV, 0);
    setMotorPWM(PWM_CH_M4_A, PWM_CH_M4_B, MOTOR4_INV, 0);
    motors_stopped = true;
    return;
  }                                         // ← ปิดแค่ if block

  float Vx = motor_msg.linear.x;
  float Wz = motor_msg.angular.z;

  motors_stopped = (fabsf(Vx) < 0.01f && fabsf(Wz) < 0.01f);

  float left_cmd  = (Vx - Wz * LR_WHEELS_DISTANCE * 0.5f) / MAX_LINEAR_SPEED;
  float right_cmd = (Vx + Wz * LR_WHEELS_DISTANCE * 0.5f) / MAX_LINEAR_SPEED;

  left_cmd  = constrain(left_cmd,  -MAX_RPM_RATIO, MAX_RPM_RATIO);
  right_cmd = constrain(right_cmd, -MAX_RPM_RATIO, MAX_RPM_RATIO);

  bool is_straight = (fabsf(Wz) < 0.05f && fabsf(Vx) > 0.01f);
  float correction = 0;

  if (is_straight)
  {
    int32_t left_ticks  = motor1_encoder_ticks + motor3_encoder_ticks;
    int32_t right_ticks = motor2_encoder_ticks + motor4_encoder_ticks;

    int32_t d_left  = left_ticks  - prev_left_ticks;
    int32_t d_right = right_ticks - prev_right_ticks;

    prev_left_ticks  = left_ticks;
    prev_right_ticks = right_ticks;

    float RIGHT_SCALE = 0.27;
    float left_speed  = (float)d_left;
    float right_speed = (float)d_right * RIGHT_SCALE;
    float error = right_speed - left_speed;

    filtered_error = ENCODER_FILTER * filtered_error +
                     (1.0f - ENCODER_FILTER) * error;

    integral_error += filtered_error;
    integral_error = constrain(integral_error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float derivative = filtered_error - prev_error;
    prev_error = filtered_error;

    correction = KP_STRAIGHT * filtered_error
               + KI_STRAIGHT * integral_error
               + KD_STRAIGHT * derivative;
    correction = constrain(correction, -0.3f, 0.3f);

    left_cmd  -= correction;
    right_cmd += correction;
    left_cmd  = constrain(left_cmd,  -MAX_RPM_RATIO, MAX_RPM_RATIO);
    right_cmd = constrain(right_cmd, -MAX_RPM_RATIO, MAX_RPM_RATIO);

    debug_dl_msg.data   = (float)abs(d_left);
    debug_dr_msg.data   = (float)abs(d_right);
    debug_err_msg.data  = filtered_error;
    debug_corr_msg.data = correction;

    rcl_publish(&debug_dl_publisher,   &debug_dl_msg,   NULL);
    rcl_publish(&debug_dr_publisher,   &debug_dr_msg,   NULL);
    rcl_publish(&debug_err_publisher,  &debug_err_msg,  NULL);
    rcl_publish(&debug_corr_publisher, &debug_corr_msg, NULL);
  }
  else
  {
    prev_left_ticks  = motor1_encoder_ticks + motor3_encoder_ticks;
    prev_right_ticks = motor2_encoder_ticks + motor4_encoder_ticks;
    filtered_error = 0;
    prev_error     = 0;
    integral_error = 0;
  }                                         

  float LEFT_BIAS  = 1.00f;
  float RIGHT_BIAS = 1.2f;

  setMotorPWM(PWM_CH_M1_A, PWM_CH_M1_B, MOTOR1_INV, left_cmd  * LEFT_BIAS);
  setMotorPWM(PWM_CH_M3_A, PWM_CH_M3_B, MOTOR3_INV, left_cmd  * LEFT_BIAS);
  setMotorPWM(PWM_CH_M2_A, PWM_CH_M2_B, MOTOR2_INV, right_cmd * RIGHT_BIAS);
  setMotorPWM(PWM_CH_M4_A, PWM_CH_M4_B, MOTOR4_INV, right_cmd * RIGHT_BIAS);
}                           

// ================= MOTOR PWM (MDD3A) =================

void setMotorPWM(int ch_a, int ch_b, bool invert, float speed)
{
  if (invert) speed = -speed;

  if (speed > 0.001f)
  {
    ledcWrite(ch_a, 255);   // full forward
    ledcWrite(ch_b, 0);
  }
  else if (speed < -0.001f)
  {
    ledcWrite(ch_a, 0);
    ledcWrite(ch_b, 255);   // full backward
  }
  else
  {
    ledcWrite(ch_a, 0);
    ledcWrite(ch_b, 0);
  }
}

// ================= ENCODER DISTANCE =================

void Encoder()
{
  static int32_t prev1=0, prev2=0, prev3=0, prev4=0;

  int32_t t1 = motor1_encoder_ticks;
  int32_t t2 = motor2_encoder_ticks;
  int32_t t3 = motor3_encoder_ticks;
  int32_t t4 = motor4_encoder_ticks;

  int32_t d1 = t1 - prev1;
  int32_t d2 = t2 - prev2;
  int32_t d3 = t3 - prev3;
  int32_t d4 = t4 - prev4;

  prev1 = t1; prev2 = t2; prev3 = t3; prev4 = t4;

  // ถ้า motor หยุด → ไม่นับระยะเลย ป้องกัน noise drift
  if (motors_stopped) return;

  // กรอง noise: ticks น้อยกว่า threshold ต่อ cycle = noise → ไม่นับ
  if (abs(d1) < TICK_THRESHOLD) d1 = 0;
  if (abs(d2) < TICK_THRESHOLD) d2 = 0;
  if (abs(d3) < TICK_THRESHOLD) d3 = 0;
  if (abs(d4) < TICK_THRESHOLD) d4 = 0;

  float avg_ticks = (d1 + d2 + d3 + d4) / 4.0f;
  float rev       = avg_ticks / ENCODER_TICKS_PER_REV;

  distance_inside_planter += rev * WHEEL_CIRCUMFERENCE * 0.641f;  //callibrate dist. w/dashboard
}

// ================= PUBLISH =================

void publishData()
{
  distance_msg.data = distance_inside_planter;
  rcl_publish(&distance_publisher, &distance_msg, NULL);
}

// ================= createEntities =================

bool createEntities()
{
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 77);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, "quin_robot_node", "", &support);

  rclc_publisher_init_best_effort(
    &distance_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/quin/distance_inside_planter");

  rclc_publisher_init_best_effort(
    &debug_dl_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/quin/debug/d_left");

  rclc_publisher_init_best_effort(
    &debug_dr_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/quin/debug/d_right");

  rclc_publisher_init_best_effort(
    &debug_err_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/quin/debug/error");

  rclc_publisher_init_best_effort(
    &debug_corr_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/quin/debug/correction");

  rclc_publisher_init_best_effort(
    &robot_ready_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/quin/robot_ready");

  rclc_subscription_init_best_effort(
    &motor_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/quin/move_command");

  rclc_subscription_init_best_effort(
    &reset_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/quin/trigger_reset");

  rclc_timer_init_default(
    &control_timer, &support,
    RCL_MS_TO_NS(CONTROL_PERIOD),
    controlCallback);

  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 3, &allocator);

  rclc_executor_add_subscription(
    &executor, &motor_subscriber, &motor_msg, &twistCallback, ON_NEW_DATA);

  rclc_executor_add_subscription(
    &executor, &reset_subscriber, &reset_msg, &resetCallback, ON_NEW_DATA);

  rclc_executor_add_timer(&executor, &control_timer);

  return true;
}

// ================= destroyEntities =================

bool destroyEntities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&distance_publisher,   &node);
  rcl_publisher_fini(&debug_dl_publisher,   &node);
  rcl_publisher_fini(&debug_dr_publisher,   &node);
  rcl_publisher_fini(&debug_err_publisher,  &node);
  rcl_publisher_fini(&debug_corr_publisher, &node);
  rcl_publisher_fini(&robot_ready_publisher, &node);
  rcl_subscription_fini(&motor_subscriber,  &node);
  rcl_subscription_fini(&reset_subscriber,  &node);
  rcl_timer_fini(&control_timer);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  return true;
}