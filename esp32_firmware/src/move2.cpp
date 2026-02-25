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

#include "../config/move.h"

// -------- Helpers --------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

// -------- Command Timeout --------
#define CMD_TIMEOUT_MS 500      // stop motors if no command received for 500ms

// -------- ROS entities --------

rcl_publisher_t debug_motor_publisher;
geometry_msgs__msg__Twist debug_motor_msg;

rcl_publisher_t debug_encoder_publisher;
geometry_msgs__msg__Twist debug_encoder_msg;

rcl_publisher_t distance_publisher;
std_msgs__msg__Float32 distance_msg;

rcl_subscription_t motor_subscriber;
geometry_msgs__msg__Twist motor_msg;

rcl_subscription_t reset_subscriber;           // subscribes /quin/reset_distance
geometry_msgs__msg__Twist reset_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

// -------- Distance tracking --------
float distance_inside_planter = 0.0f;
const float WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;

// -------- Command timeout tracking --------
unsigned long prev_cmd_time = 0;

// -------- Encoder variables --------
volatile int32_t motor1_encoder_ticks = 0;
volatile int32_t motor2_encoder_ticks = 0;
volatile int32_t motor3_encoder_ticks = 0;
volatile int32_t motor4_encoder_ticks = 0;

// Both RISING and FALLING edges for better accuracy
void IRAM_ATTR handleEncoder_MOTOR1_A() { motor1_encoder_ticks += digitalRead(MOTOR1_ENCODER_PIN_B) ? 1 : -1; }
void IRAM_ATTR handleEncoder_MOTOR2_A() { motor2_encoder_ticks += digitalRead(MOTOR2_ENCODER_PIN_B) ? 1 : -1; }
void IRAM_ATTR handleEncoder_MOTOR3_A() { motor3_encoder_ticks += digitalRead(MOTOR3_ENCODER_PIN_B) ? 1 : -1; }
void IRAM_ATTR handleEncoder_MOTOR4_A() { motor4_encoder_ticks += digitalRead(MOTOR4_ENCODER_PIN_B) ? 1 : -1; }

// Channel B interrupts (CHANGE) — inverted logic to keep consistent direction
void IRAM_ATTR handleEncoder_MOTOR1_B() { motor1_encoder_ticks += digitalRead(MOTOR1_ENCODER_PIN_A) ? -1 : 1; }
void IRAM_ATTR handleEncoder_MOTOR2_B() { motor2_encoder_ticks += digitalRead(MOTOR2_ENCODER_PIN_A) ? -1 : 1; }
void IRAM_ATTR handleEncoder_MOTOR3_B() { motor3_encoder_ticks += digitalRead(MOTOR3_ENCODER_PIN_A) ? -1 : 1; }
void IRAM_ATTR handleEncoder_MOTOR4_B() { motor4_encoder_ticks += digitalRead(MOTOR4_ENCODER_PIN_A) ? -1 : 1; }

// -------- Function declarations --------
void rclErrorLoop();
bool createEntities();
bool destroyEntities();
void publishData();
void Encoder();
void driveDifferential();
void setMotor(int a, int b, float rpm);

// -------------------- SETUP --------------------

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // Motor output pins
    pinMode(MOTOR1_IN_A, OUTPUT);
    pinMode(MOTOR1_IN_B, OUTPUT);
    pinMode(MOTOR2_IN_A, OUTPUT);
    pinMode(MOTOR2_IN_B, OUTPUT);
    pinMode(MOTOR3_IN_A, OUTPUT);
    pinMode(MOTOR3_IN_B, OUTPUT);
    pinMode(MOTOR4_IN_A, OUTPUT);
    pinMode(MOTOR4_IN_B, OUTPUT);

    // Make sure all motors are stopped at boot
    digitalWrite(MOTOR1_IN_A, LOW); digitalWrite(MOTOR1_IN_B, LOW);
    digitalWrite(MOTOR2_IN_A, LOW); digitalWrite(MOTOR2_IN_B, LOW);
    digitalWrite(MOTOR3_IN_A, LOW); digitalWrite(MOTOR3_IN_B, LOW);
    digitalWrite(MOTOR4_IN_A, LOW); digitalWrite(MOTOR4_IN_B, LOW);

    // Encoder input pins
    pinMode(MOTOR1_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(MOTOR1_ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(MOTOR2_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(MOTOR2_ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(MOTOR3_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(MOTOR3_ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(MOTOR4_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(MOTOR4_ENCODER_PIN_B, INPUT_PULLUP);

    // Attach both channels on CHANGE for full quadrature (2x resolution)
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

// -------------------- LOOP --------------------

void loop()
{
    switch (state)
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(1500,
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 10))
                    ? AGENT_AVAILABLE : WAITING_AGENT;
            );
            break;

        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroyEntities();      // FIX: clean up on partial failure
            }
            break;

        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(500,
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 10))
                    ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            );
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
            }
            break;

        case AGENT_DISCONNECTED:
            // FIX: stop motors immediately on disconnect
            digitalWrite(MOTOR1_IN_A, LOW); digitalWrite(MOTOR1_IN_B, LOW);
            digitalWrite(MOTOR2_IN_A, LOW); digitalWrite(MOTOR2_IN_B, LOW);
            digitalWrite(MOTOR3_IN_A, LOW); digitalWrite(MOTOR3_IN_B, LOW);
            digitalWrite(MOTOR4_IN_A, LOW); digitalWrite(MOTOR4_IN_B, LOW);
            destroyEntities();
            state = WAITING_AGENT;
            break;
    }
}

// -------------------- ROS CALLBACKS --------------------

void twistCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;
    motor_msg = *msg;
    prev_cmd_time = millis();       // FIX: update timestamp on every command
}

void resetCallback(const void *msgin)
{
    // Any message on /quin/reset_distance resets the odometer
    distance_inside_planter = 0.0f;
}

// -------------------- CONTROL TIMER --------------------

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        Encoder();
        driveDifferential();
        publishData();
    }
}

// -------------------- MOTOR DRIVER --------------------

void setMotor(int a, int b, float rpm)
{
    if (rpm > 0.0f) {
        digitalWrite(a, HIGH);
        digitalWrite(b, LOW);
    } else if (rpm < 0.0f) {
        digitalWrite(a, LOW);
        digitalWrite(b, HIGH);
    } else {
        digitalWrite(a, LOW);
        digitalWrite(b, LOW);
    }
}

// -------------------- DRIVE --------------------

void driveDifferential()
{
    // FIX: command timeout safety stop
    if (millis() - prev_cmd_time > CMD_TIMEOUT_MS) {
        setMotor(MOTOR1_IN_A, MOTOR1_IN_B, 0);
        setMotor(MOTOR2_IN_A, MOTOR2_IN_B, 0);
        setMotor(MOTOR3_IN_A, MOTOR3_IN_B, 0);
        setMotor(MOTOR4_IN_A, MOTOR4_IN_B, 0);
        debug_motor_msg.linear.x = 0;
        debug_motor_msg.linear.y = 0;
        debug_motor_msg.linear.z = 0;
        debug_motor_msg.angular.z = 0;
        return;
    }

    const float WHEEL_RADIUS = WHEEL_DIAMETER / 2.0f;

    float Vx = motor_msg.linear.x;
    float Wz = -motor_msg.angular.z;

    // Differential drive kinematics
    float wheel_left  = Vx - (Wz * LR_WHEELS_DISTANCE * 0.5f);
    float wheel_right = Vx + (Wz * LR_WHEELS_DISTANCE * 0.5f);

    // m/s → RPM (used for direction sign only since motors are on/off)
    float left_rpm  = (wheel_left  / (2.0f * M_PI * WHEEL_RADIUS)) * 60.0f;
    float right_rpm = (wheel_right / (2.0f * M_PI * WHEEL_RADIUS)) * 60.0f;

    setMotor(MOTOR1_IN_A, MOTOR1_IN_B, left_rpm);
    setMotor(MOTOR2_IN_A, MOTOR2_IN_B, right_rpm);
    setMotor(MOTOR3_IN_A, MOTOR3_IN_B, left_rpm);
    setMotor(MOTOR4_IN_A, MOTOR4_IN_B, right_rpm);

    debug_motor_msg.linear.x  = motor_msg.linear.x;
    debug_motor_msg.angular.z = motor_msg.angular.z;
    debug_motor_msg.linear.y  = left_rpm;
    debug_motor_msg.linear.z  = right_rpm;
}

// -------------------- ENCODER --------------------

void Encoder()
{
    static int32_t prev1 = 0, prev2 = 0, prev3 = 0, prev4 = 0;

    // Snapshot ticks (atomic on ESP32 for 32-bit values)
    int32_t t1 = motor1_encoder_ticks;
    int32_t t2 = motor2_encoder_ticks;
    int32_t t3 = motor3_encoder_ticks;
    int32_t t4 = motor4_encoder_ticks;

    int32_t d1 = t1 - prev1;
    int32_t d2 = t2 - prev2;
    int32_t d3 = t3 - prev3;
    int32_t d4 = t4 - prev4;

    prev1 = t1;
    prev2 = t2;
    prev3 = t3;
    prev4 = t4;

    // FIX: separate left/right averaging before combining
    // Motor 1 & 3 = left side, Motor 2 & 4 = right side
    float avg_left  = (d1 + d3) / 2.0f;
    float avg_right = (d2 + d4) / 2.0f;

    // FIX: only accumulate linear distance, not turning motion
    // During a turn avg_left and avg_right cancel out correctly here
    float avg_ticks    = (avg_left + avg_right) / 2.0f;
    float revolutions  = avg_ticks / (float)ENCODER_TICKS_PER_REV;
    float delta_distance = revolutions * WHEEL_CIRCUMFERENCE;

    distance_inside_planter += delta_distance;

    // Debug: publish raw delta ticks per motor
    debug_encoder_msg.linear.x  = d1;
    debug_encoder_msg.linear.y  = d2;
    debug_encoder_msg.linear.z  = d3;
    debug_encoder_msg.angular.z = d4;
}

// -------------------- PUBLISH --------------------

void publishData()
{
    distance_msg.data = distance_inside_planter;
    RCSOFTCHECK(rcl_publish(&distance_publisher, &distance_msg, NULL));
    RCSOFTCHECK(rcl_publish(&debug_encoder_publisher, &debug_encoder_msg, NULL));
    RCSOFTCHECK(rcl_publish(&debug_motor_publisher, &debug_motor_msg, NULL));
}

// -------------------- CREATE ENTITIES --------------------

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 77));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    RCCHECK(rclc_node_init_default(&node, "quin_robot_node", "", &support));

    // Publishers
    RCCHECK(rclc_publisher_init_best_effort(
        &debug_motor_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/debug/motor"));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_encoder_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/debug/encoder"));

    RCCHECK(rclc_publisher_init_best_effort(
        &distance_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/quin/distance_inside_planter"));

    // Subscribers
    RCCHECK(rclc_subscription_init_best_effort(
        &motor_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/cmd_move"));

    RCCHECK(rclc_subscription_init_best_effort(
        &reset_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/reset_distance"));     // FIX: renamed from cmd_encoder — clearer intent

    // Timer (20ms = 50Hz control loop)
    RCCHECK(rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(20),
        controlCallback));

    // Executor — handles: 2 subscriptions + 1 timer = 3 handles
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor, &motor_subscriber, &motor_msg, &twistCallback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor, &reset_subscriber, &reset_msg, &resetCallback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // Init messages
    geometry_msgs__msg__Twist__init(&debug_motor_msg);
    geometry_msgs__msg__Twist__init(&debug_encoder_msg);
    std_msgs__msg__Float32__init(&distance_msg);

    return true;
}

// -------------------- DESTROY ENTITIES --------------------

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // FIX: destroy ALL entities (publishers, subscribers, timer, node, executor, support)
    RCSOFTCHECK(rcl_publisher_fini(&debug_motor_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&debug_encoder_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&distance_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&motor_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&reset_subscriber, &node));
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rclc_support_fini(&support));

    return true;
}

// -------------------- ERROR --------------------

void rclErrorLoop()
{
    // Stop motors before restarting
    digitalWrite(MOTOR1_IN_A, LOW); digitalWrite(MOTOR1_IN_B, LOW);
    digitalWrite(MOTOR2_IN_A, LOW); digitalWrite(MOTOR2_IN_B, LOW);
    digitalWrite(MOTOR3_IN_A, LOW); digitalWrite(MOTOR3_IN_B, LOW);
    digitalWrite(MOTOR4_IN_A, LOW); digitalWrite(MOTOR4_IN_B, LOW);
    delay(100);
    ESP.restart();
}