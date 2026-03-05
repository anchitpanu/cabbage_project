/**
 * esp32_planting.cpp  —  ESP32 #2  Planting Motor Driver
 *                        + Overcurrent Protection
 *
 * ── Commands  /microros/actuator_cmd  (std_msgs/Int32) ───────────────
 *   1 = GRIPPER_OPEN    4 = FEEDER_CLOSE
 *   2 = GRIPPER_CLOSE   5 = STEPPER_DOWN
 *   3 = FEEDER_OPEN     6 = STEPPER_UP
 *
 * ── Published Topics ─────────────────────────────────────────────────
 *   /microros/actuator_ack    std_msgs/Bool   true = done, false = error
 *   /microros/actuator_error  std_msgs/Int32  error code (see ERROR_* below)
 *
 * ── Overcurrent behaviour ────────────────────────────────────────────
 *   1st detection → stop motor, wait RETRY_DELAY_MS, retry once
 *   2nd detection → halt permanently, publish error to Pi
 *
 * ── Sensor ───────────────────────────────────────────────────────────
 *   INA219 x3 on I2C (SDA→GPIO21, SCL→GPIO22)
 *     Stepper → address 0x40
 *     Gripper → address 0x41
 *     Feeder  → address 0x44
 */

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <ESP32Servo.h>
#include <Adafruit_INA219.h>

// ═══════════════════════════════════════════════════════════════════
// ★  MOTOR CURRENT LIMITS  ← set your values here
// ═══════════════════════════════════════════════════════════════════
constexpr float STEPPER_MAX_CURRENT_A = 1.5f;   // e.g. NEMA17 rated 1.0A → limit 1.5A
constexpr float GRIPPER_MAX_CURRENT_A = 0.8f;   // e.g. MG996R stall ~0.5A → limit 0.8A
constexpr float FEEDER_MAX_CURRENT_A  = 0.8f;

// Convert A → mA for internal use
constexpr float STEPPER_MAX_MA = STEPPER_MAX_CURRENT_A * 1000.0f;
constexpr float GRIPPER_MAX_MA = GRIPPER_MAX_CURRENT_A * 1000.0f;
constexpr float FEEDER_MAX_MA  = FEEDER_MAX_CURRENT_A  * 1000.0f;

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
constexpr int32_t ERR_NONE             = 0;
constexpr int32_t ERR_OVERCURRENT_STEP = 10;
constexpr int32_t ERR_OVERCURRENT_GRIP = 11;
constexpr int32_t ERR_OVERCURRENT_FEED = 12;
constexpr int32_t ERR_SENSOR_FAIL      = 20;

// ═══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════
constexpr uint8_t PIN_STEP    = 25;
constexpr uint8_t PIN_DIR     = 26;
constexpr uint8_t PIN_EN      = 27;
constexpr uint8_t PIN_GRIPPER = 18;
constexpr uint8_t PIN_FEEDER  = 19;
constexpr uint8_t PIN_LED     = LED_BUILTIN;

// ═══════════════════════════════════════════════════════════════════
// OTHER PARAMETERS
// ═══════════════════════════════════════════════════════════════════
constexpr uint32_t STEPS_DOWN             = 400;
constexpr uint32_t STEP_DELAY_US          = 800;

constexpr uint8_t  GRIPPER_OPEN_DEG       = 80;
constexpr uint8_t  GRIPPER_CLOSE_DEG      = 10;
constexpr uint8_t  FEEDER_OPEN_DEG        = 70;
constexpr uint8_t  FEEDER_CLOSE_DEG       = 10;

constexpr uint32_t SERVO_SETTLE_MS        = 300;   // wait after servo write
constexpr uint32_t STEPPER_SAMPLE_EVERY_N = 20;    // check current every N steps
constexpr uint32_t RETRY_DELAY_MS         = 500;   // pause before retry

// ═══════════════════════════════════════════════════════════════════
// HARDWARE
// ═══════════════════════════════════════════════════════════════════
static Servo           gripper;
static Servo           feeder;
static Adafruit_INA219 ina_stepper(0x40);
static Adafruit_INA219 ina_gripper(0x41);
static Adafruit_INA219 ina_feeder (0x44);

// ═══════════════════════════════════════════════════════════════════
// RUNTIME STATE
// ═══════════════════════════════════════════════════════════════════
static bool          g_busy        = false;
static bool          g_halted      = false;
static unsigned long g_settle_end  = 0;
static int           g_current_cmd = 0;
static uint8_t       g_retry_count = 0;   // retry counter for current command

// ═══════════════════════════════════════════════════════════════════
// micro-ROS
// ═══════════════════════════════════════════════════════════════════
static rcl_subscription_t  sub_cmd;
static rcl_publisher_t     pub_ack;
static rcl_publisher_t     pub_error;
static std_msgs__msg__Int32 msg_cmd;
static std_msgs__msg__Bool  msg_ack;
static std_msgs__msg__Int32 msg_error;
static rclc_executor_t     executor;
static rclc_support_t      support;
static rcl_allocator_t     allocator;
static rcl_node_t          node;

// ═══════════════════════════════════════════════════════════════════
// ERROR HANDLER
// ═══════════════════════════════════════════════════════════════════
#define RCCHECK(fn) \
    do { \
        if ((fn) != RCL_RET_OK) { \
            while (true) { \
                digitalWrite(PIN_LED, !digitalRead(PIN_LED)); \
                delay(150); \
            } \
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

static void fatalHalt(int32_t err_code) {
    digitalWrite(PIN_EN, HIGH);   // disable stepper driver
    g_halted = true;
    g_busy   = false;
    publishAck(false);
    publishError(err_code);
    digitalWrite(PIN_LED, HIGH);  // LED stays ON = fault
}

// Read current (mA) for each motor
static float readCurrentMA(int cmd) {
    if (cmd == CMD_STEPPER_DOWN || cmd == CMD_STEPPER_UP)
        return ina_stepper.getCurrent_mA();
    if (cmd == CMD_GRIPPER_OPEN || cmd == CMD_GRIPPER_CLOSE)
        return ina_gripper.getCurrent_mA();
    if (cmd == CMD_FEEDER_OPEN  || cmd == CMD_FEEDER_CLOSE)
        return ina_feeder.getCurrent_mA();
    return 0.0f;
}

static float limitForCmd(int cmd) {
    if (cmd == CMD_STEPPER_DOWN || cmd == CMD_STEPPER_UP)  return STEPPER_MAX_MA;
    if (cmd == CMD_GRIPPER_OPEN || cmd == CMD_GRIPPER_CLOSE) return GRIPPER_MAX_MA;
    if (cmd == CMD_FEEDER_OPEN  || cmd == CMD_FEEDER_CLOSE)  return FEEDER_MAX_MA;
    return 9999.0f;
}

static int32_t errorCodeForCmd(int cmd) {
    if (cmd == CMD_STEPPER_DOWN || cmd == CMD_STEPPER_UP)    return ERR_OVERCURRENT_STEP;
    if (cmd == CMD_GRIPPER_OPEN || cmd == CMD_GRIPPER_CLOSE) return ERR_OVERCURRENT_GRIP;
    return ERR_OVERCURRENT_FEED;
}

// ═══════════════════════════════════════════════════════════════════
// STEPPER MOVE — checks current every STEPPER_SAMPLE_EVERY_N steps
// Returns true = success, false = fatal overcurrent
// ═══════════════════════════════════════════════════════════════════
static bool stepperMoveProtected(bool down) {
    digitalWrite(PIN_DIR, down ? HIGH : LOW);
    digitalWrite(PIN_EN,  LOW);

    for (uint32_t i = 0; i < STEPS_DOWN; ++i) {
        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(STEP_DELAY_US);
        digitalWrite(PIN_STEP, LOW);
        delayMicroseconds(STEP_DELAY_US);

        if ((i % STEPPER_SAMPLE_EVERY_N) == 0) {
            float current_ma = ina_stepper.getCurrent_mA();
            Serial.printf("[STEP] i=%d current=%.1fmA limit=%.1fmA\n",
                          i, current_ma, STEPPER_MAX_MA);

            if (current_ma > STEPPER_MAX_MA) {
                digitalWrite(PIN_EN, HIGH);  // stop immediately

                if (g_retry_count < 1) {
                    g_retry_count++;
                    Serial.printf("[OC] Stepper overcurrent! Retrying in %dms...\n",
                                  RETRY_DELAY_MS);
                    delay(RETRY_DELAY_MS);
                    return stepperMoveProtected(down);   // retry once
                }

                Serial.println("[OC] Stepper overcurrent FATAL");
                return false;
            }
        }
    }

    digitalWrite(PIN_EN, HIGH);
    return true;
}

// ═══════════════════════════════════════════════════════════════════
// EXECUTE COMMAND
// ═══════════════════════════════════════════════════════════════════
static void executeCommand(int cmd) {
    g_current_cmd = cmd;
    g_retry_count = 0;

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
            if (!stepperMoveProtected(true)) {
                fatalHalt(ERR_OVERCURRENT_STEP);
                return;
            }
            g_settle_end = millis();
            break;

        case CMD_STEPPER_UP:
            if (!stepperMoveProtected(false)) {
                fatalHalt(ERR_OVERCURRENT_STEP);
                return;
            }
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
    if (g_busy || g_halted) return;

    const int cmd = static_cast<const std_msgs__msg__Int32*>(msgin)->data;
    g_busy = true;
    digitalWrite(PIN_LED, HIGH);
    Serial.printf("[CMD] Received command %d\n", cmd);
    executeCommand(cmd);
}

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    pinMode(PIN_LED,  OUTPUT);
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR,  OUTPUT);
    pinMode(PIN_EN,   OUTPUT);
    digitalWrite(PIN_EN, HIGH);

    // Servos → home
    gripper.attach(PIN_GRIPPER, 500, 2400);
    feeder.attach(PIN_FEEDER,   500, 2400);
    gripper.write(GRIPPER_CLOSE_DEG);
    feeder.write(FEEDER_CLOSE_DEG);
    delay(700);

    // Current sensors
    bool sensors_ok = ina_stepper.begin() &&
                      ina_gripper.begin() &&
                      ina_feeder.begin();

    if (!sensors_ok) {
        Serial.println("[ERROR] INA219 init failed! Check wiring + I2C addresses.");
        g_halted = true;
    } else {
        Serial.printf("[OK] Current limits — stepper:%.1fA gripper:%.1fA feeder:%.1fA\n",
                      STEPPER_MAX_CURRENT_A, GRIPPER_MAX_CURRENT_A, FEEDER_MAX_CURRENT_A);
    }

    // micro-ROS
    set_microros_serial_transports(Serial);
    delay(2000);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_planting", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/microros/actuator_cmd"));

    RCCHECK(rclc_publisher_init_default(
        &pub_ack, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/microros/actuator_ack"));

    RCCHECK(rclc_publisher_init_default(
        &pub_error, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/microros/actuator_error"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &sub_cmd, &msg_cmd, &cbCmd, ON_NEW_DATA));

    if (g_halted) publishError(ERR_SENSOR_FAIL);

    // Ready blink
    for (int i = 0; i < 3; ++i) {
        digitalWrite(PIN_LED, HIGH); delay(120);
        digitalWrite(PIN_LED, LOW);  delay(120);
    }
    if (g_halted) digitalWrite(PIN_LED, HIGH);
}

// ═══════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

    if (!g_busy || g_halted) return;

    // Wait for servo settle, then check current
    if (millis() < g_settle_end) return;

    float current_ma = readCurrentMA(g_current_cmd);
    float limit_ma   = limitForCmd(g_current_cmd);

    Serial.printf("[MON] cmd=%d current=%.1fmA limit=%.1fmA\n",
                  g_current_cmd, current_ma, limit_ma);

    if (current_ma > limit_ma) {
        if (g_retry_count < 1) {
            // Retry once
            g_retry_count++;
            Serial.printf("[OC] Overcurrent! Retrying cmd=%d in %dms\n",
                          g_current_cmd, RETRY_DELAY_MS);
            delay(RETRY_DELAY_MS);
            executeCommand(g_current_cmd);   // retry same command
            return;
        }
        // Fatal
        Serial.printf("[OC] FATAL overcurrent on cmd=%d\n", g_current_cmd);
        fatalHalt(errorCodeForCmd(g_current_cmd));
        return;
    }

    // All good
    publishAck(true);
    g_busy = false;
    digitalWrite(PIN_LED, LOW);
    Serial.printf("[OK] cmd=%d done\n", g_current_cmd);
}