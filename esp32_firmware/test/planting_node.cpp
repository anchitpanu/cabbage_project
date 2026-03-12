/**
 * esp32_planting.cpp  —  ESP32 #2  Planting Motor Driver
 *
 * ── Commands  /microros/actuator_cmd  (std_msgs/Int32) ───────────────
 *   1 = GRIPPER_OPEN    4 = FEEDER_CLOSE
 *   2 = GRIPPER_CLOSE   5 = STEPPER_DOWN
 *   3 = FEEDER_OPEN     6 = STEPPER_UP
 *
 * ── Published Topics ─────────────────────────────────────────────────
 *   /microros/actuator_ack    std_msgs/Bool   true = done, false = error
 *   /microros/actuator_error  std_msgs/Int32  error code (see ERROR_* below)
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <ESP32Servo.h>

// ═══════════════════════════════════════════════════════════════════
// COMMAND IDs
// ═══════════════════════════════════════════════════════════════════
constexpr int8_t CMD_GRIPPER_OPEN  = 1;
constexpr int8_t CMD_GRIPPER_CLOSE = 2;
constexpr int8_t CMD_FEEDER_OPEN   = 3;
constexpr int8_t CMD_FEEDER_CLOSE  = 4;
constexpr int8_t CMD_STEPPER_DOWN  = 5;
constexpr int8_t CMD_STEPPER_UP    = 6;

// ═══════════════════════════════════════════════════════════════════
// ERROR CODES
// ═══════════════════════════════════════════════════════════════════
constexpr int32_t ERR_NONE = 0;

// ═══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════
constexpr uint8_t PIN_STEP    = 26;
constexpr uint8_t PIN_DIR     = 27;
constexpr uint8_t PIN_GRIPPER = 16;
constexpr uint8_t PIN_FEEDER  = 19;

// ═══════════════════════════════════════════════════════════════════
// STEPPER PARAMETERS
// ═══════════════════════════════════════════════════════════════════
constexpr uint32_t STEPS_DOWN    = 4750;   // ระยะลง
constexpr uint32_t STEPS_UP      = 4600;   // ระยะขึ้น
constexpr uint32_t STEP_DELAY_US = 700;    // ความเร็ว

// ═══════════════════════════════════════════════════════════════════
// SERVO PARAMETERS
// ═══════════════════════════════════════════════════════════════════
constexpr uint8_t GRIPPER_OPEN_DEG  = 90;
constexpr uint8_t GRIPPER_CLOSE_DEG = 135;
constexpr uint8_t FEEDER_OPEN_DEG   = 180;
constexpr uint8_t FEEDER_CLOSE_DEG  = 90;

constexpr uint32_t SERVO_SETTLE_MS = 1000;

// ═══════════════════════════════════════════════════════════════════
// HARDWARE
// ═══════════════════════════════════════════════════════════════════
static Servo gripper;
static Servo feeder;

// ═══════════════════════════════════════════════════════════════════
// RUNTIME STATE
// ═══════════════════════════════════════════════════════════════════
static bool g_busy = false;
static unsigned long g_settle_end = 0;
static int g_current_cmd = 0;

// ═══════════════════════════════════════════════════════════════════
// micro-ROS
// ═══════════════════════════════════════════════════════════════════
static rcl_subscription_t sub_cmd;
static rcl_publisher_t pub_ack;
static rcl_publisher_t pub_error;

static std_msgs__msg__Int32 msg_cmd;
static std_msgs__msg__Bool msg_ack;
static std_msgs__msg__Int32 msg_error;

static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;

// ═══════════════════════════════════════════════════════════════════
// ERROR HANDLER
// ═══════════════════════════════════════════════════════════════════
#define RCCHECK(fn) \
  do { \
    if ((fn) != RCL_RET_OK) { \
      while (true) { delay(150); } \
    } \
  } while (0)

// ═══════════════════════════════════════════════════════════════════
// HELPERS
// ═══════════════════════════════════════════════════════════════════
static void publishAck(bool ok) {
  msg_ack.data = ok;
  rcl_publish(&pub_ack, &msg_ack, nullptr);
}

static void publishError(int32_t code) {
  msg_error.data = code;
  rcl_publish(&pub_error, &msg_error, nullptr);
  Serial.printf("[ERROR] code=%d\n", code);
}

// ═══════════════════════════════════════════════════════════════════
// STEPPER MOVE
// ═══════════════════════════════════════════════════════════════════
static void stepperMove(bool down) {

  digitalWrite(PIN_DIR, down ? HIGH : LOW);

  uint32_t steps = down ? STEPS_DOWN : STEPS_UP;

  for (uint32_t i = 0; i < steps; ++i) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// ═══════════════════════════════════════════════════════════════════
// EXECUTE COMMAND
// ═══════════════════════════════════════════════════════════════════
static void executeCommand(int cmd) {

  g_current_cmd = cmd;

  switch (cmd) {

    case CMD_GRIPPER_OPEN:
      gripper.write(GRIPPER_OPEN_DEG);
      g_settle_end = millis() + SERVO_SETTLE_MS;
      break;

    case CMD_GRIPPER_CLOSE:
      gripper.write(GRIPPER_CLOSE_DEG);
      g_settle_end = millis() + SERVO_SETTLE_MS;
      break;

    case CMD_FEEDER_OPEN:
      feeder.write(FEEDER_OPEN_DEG);
      g_settle_end = millis() + SERVO_SETTLE_MS;
      break;

    case CMD_FEEDER_CLOSE:
      feeder.write(FEEDER_CLOSE_DEG);
      g_settle_end = millis() + SERVO_SETTLE_MS;
      break;

    case CMD_STEPPER_DOWN:
      stepperMove(true);
      g_settle_end = millis();
      break;

    case CMD_STEPPER_UP:
      stepperMove(false);
      g_settle_end = millis();
      break;

    default:
      g_busy = false;
      return;
  }
}

// ═══════════════════════════════════════════════════════════════════
// micro-ROS CALLBACK
// ═══════════════════════════════════════════════════════════════════
static void cbCmd(const void* msgin) {

  if (g_busy) return;

  const int cmd = static_cast<const std_msgs__msg__Int32*>(msgin)->data;

  g_busy = true;

  Serial.printf("[CMD] Received command %d\n", cmd);

  executeCommand(cmd);
}

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {

  Serial.begin(115200);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  gripper.attach(PIN_GRIPPER, 500, 2400);
  feeder.attach(PIN_FEEDER, 500, 2400);

  gripper.write(GRIPPER_CLOSE_DEG);
  feeder.write(FEEDER_CLOSE_DEG);

  delay(700);

  set_microros_serial_transports(Serial);

  delay(2000);

  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 77));

  RCCHECK(rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator));

  RCCHECK(rclc_node_init_default(&node, "esp32_planting", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &sub_cmd,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/microros/actuator_cmd"));

  RCCHECK(rclc_publisher_init_default(
    &pub_ack,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/microros/actuator_ack"));

  RCCHECK(rclc_publisher_init_default(
    &pub_error,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/microros/actuator_error"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &sub_cmd,
    &msg_cmd,
    &cbCmd,
    ON_NEW_DATA));

  Serial.println("[OK] esp32_planting ready");
}

// ═══════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════
void loop() {

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  if (!g_busy) return;

  if (millis() < g_settle_end) return;

  publishAck(true);

  g_busy = false;

  Serial.printf("[OK] cmd=%d done\n", g_current_cmd);
}