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
#include <std_msgs/msg/bool.h>       // FIX: added for reset topic

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
#define CMD_TIMEOUT_MS 500

// -------- PID Config --------
#define PWM_STEPS       20          // software PWM resolution (20 steps)

// Tune these values:
#define PID_KP          0.008f      // proportional gain
#define PID_KI          0.0001f     // integral gain
#define PID_KD          0.002f      // derivative gain
#define PID_MAX_CORR    1.0f        // max correction (clamp)
#define PID_INTEGRAL_MAX 100.0f     // anti-windup clamp

// -------- ROS entities --------
rcl_publisher_t debug_motor_publisher;
geometry_msgs__msg__Twist debug_motor_msg;

rcl_publisher_t debug_encoder_publisher;
geometry_msgs__msg__Twist debug_encoder_msg;

rcl_publisher_t distance_publisher;
std_msgs__msg__Float32 distance_msg;

rcl_subscription_t motor_subscriber;
geometry_msgs__msg__Twist motor_msg;

rcl_subscription_t reset_subscriber;
std_msgs__msg__Bool reset_msg;       // FIX: was Twist, now Bool

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

// -------- Command timeout --------
unsigned long prev_cmd_time = 0;

// -------- Encoder variables --------
volatile int32_t motor1_encoder_ticks = 0;
volatile int32_t motor2_encoder_ticks = 0;
volatile int32_t motor3_encoder_ticks = 0;
volatile int32_t motor4_encoder_ticks = 0;

// -------- PID state --------
int32_t  pid_left_total  = 0;
int32_t  pid_right_total = 0;
float    pid_integral    = 0.0f;
float    pid_prev_error  = 0.0f;
int      pwm_counter     = 0;
float    left_duty       = 1.0f;
float    right_duty      = 1.0f;

// -------- ISR handlers --------
void IRAM_ATTR handleEncoder_MOTOR1_A() { motor1_encoder_ticks += digitalRead(MOTOR1_ENCODER_PIN_B) ? 1 : -1; }
void IRAM_ATTR handleEncoder_MOTOR2_A() { motor2_encoder_ticks += digitalRead(MOTOR2_ENCODER_PIN_B) ? 1 : -1; }
void IRAM_ATTR handleEncoder_MOTOR3_A() { motor3_encoder_ticks += digitalRead(MOTOR3_ENCODER_PIN_B) ? 1 : -1; }
void IRAM_ATTR handleEncoder_MOTOR4_A() { motor4_encoder_ticks += digitalRead(MOTOR4_ENCODER_PIN_B) ? 1 : -1; }

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
void setMotorPWM(int a, int b, float rpm, float duty);
void resetPID();

// -------------------- SETUP --------------------

void setup()
{
    Serial.begin(115200);
    delay(2000);

    // Serial transport MUST come first
    set_microros_serial_transports(Serial);

    // Motor pins
    pinMode(MOTOR1_IN_A, OUTPUT); pinMode(MOTOR1_IN_B, OUTPUT);
    pinMode(MOTOR2_IN_A, OUTPUT); pinMode(MOTOR2_IN_B, OUTPUT);
    pinMode(MOTOR3_IN_A, OUTPUT); pinMode(MOTOR3_IN_B, OUTPUT);
    pinMode(MOTOR4_IN_A, OUTPUT); pinMode(MOTOR4_IN_B, OUTPUT);

    digitalWrite(MOTOR1_IN_A, LOW); digitalWrite(MOTOR1_IN_B, LOW);
    digitalWrite(MOTOR2_IN_A, LOW); digitalWrite(MOTOR2_IN_B, LOW);
    digitalWrite(MOTOR3_IN_A, LOW); digitalWrite(MOTOR3_IN_B, LOW);
    digitalWrite(MOTOR4_IN_A, LOW); digitalWrite(MOTOR4_IN_B, LOW);

    // Encoder pins
    pinMode(MOTOR1_ENCODER_PIN_A, INPUT_PULLUP); pinMode(MOTOR1_ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(MOTOR2_ENCODER_PIN_A, INPUT_PULLUP); pinMode(MOTOR2_ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(MOTOR3_ENCODER_PIN_A, INPUT_PULLUP); pinMode(MOTOR3_ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(MOTOR4_ENCODER_PIN_A, INPUT_PULLUP); pinMode(MOTOR4_ENCODER_PIN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_PIN_A), handleEncoder_MOTOR1_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_PIN_B), handleEncoder_MOTOR1_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCODER_PIN_A), handleEncoder_MOTOR2_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCODER_PIN_B), handleEncoder_MOTOR2_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR3_ENCODER_PIN_A), handleEncoder_MOTOR3_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR3_ENCODER_PIN_B), handleEncoder_MOTOR3_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR4_ENCODER_PIN_A), handleEncoder_MOTOR4_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR4_ENCODER_PIN_B), handleEncoder_MOTOR4_B, CHANGE);

    // Remove ALL ROS init from here — createEntities() handles it
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
            if (state == WAITING_AGENT) { destroyEntities(); }
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
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    motor_msg = *msg;
    prev_cmd_time = millis();

    // Reset PID whenever a new command comes in
    resetPID();
}

void resetCallback(const void *msgin)
{
    // FIX: cast to Bool instead of Twist
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;

    if (msg->data)
    {
        distance_inside_planter = 0.0f;

        // Also reset raw encoder ticks so PID starts clean
        noInterrupts();
        motor1_encoder_ticks = 0;
        motor2_encoder_ticks = 0;
        motor3_encoder_ticks = 0;
        motor4_encoder_ticks = 0;
        interrupts();

        resetPID();
    }
}

// -------------------- PID RESET --------------------

void resetPID()
{
    pid_left_total  = 0;
    pid_right_total = 0;
    pid_integral    = 0.0f;
    pid_prev_error  = 0.0f;
    left_duty       = 1.0f;
    right_duty      = 1.0f;
    pwm_counter     = 0;
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

// -------------------- MOTOR DRIVER (with soft PWM) --------------------

void setMotorPWM(int a, int b, float rpm, float duty)
{
    bool fire = (pwm_counter < (int)(duty * PWM_STEPS));

    if (!fire) {
        digitalWrite(a, LOW);
        digitalWrite(b, LOW);
        return;
    }

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
    // Command timeout safety stop
    if (millis() - prev_cmd_time > CMD_TIMEOUT_MS) {
        digitalWrite(MOTOR1_IN_A, LOW); digitalWrite(MOTOR1_IN_B, LOW);
        digitalWrite(MOTOR2_IN_A, LOW); digitalWrite(MOTOR2_IN_B, LOW);
        digitalWrite(MOTOR3_IN_A, LOW); digitalWrite(MOTOR3_IN_B, LOW);
        digitalWrite(MOTOR4_IN_A, LOW); digitalWrite(MOTOR4_IN_B, LOW);
        debug_motor_msg.linear.x  = 0;
        debug_motor_msg.linear.y  = 0;
        debug_motor_msg.linear.z  = 0;
        debug_motor_msg.angular.z = 0;
        resetPID();
        return;
    }

    const float WHEEL_RADIUS = WHEEL_DIAMETER / 2.0f;

    float Vx = motor_msg.linear.x;
    float Wz = -motor_msg.angular.z;

    float wheel_left  = Vx - (Wz * LR_WHEELS_DISTANCE * 0.5f);
    float wheel_right = Vx + (Wz * LR_WHEELS_DISTANCE * 0.5f);

    float left_rpm  = (wheel_left  / (2.0f * M_PI * WHEEL_RADIUS)) * 60.0f;
    float right_rpm = (wheel_right / (2.0f * M_PI * WHEEL_RADIUS)) * 60.0f;

    // ---- PID heading correction ----
    if (fabsf(Wz) < 0.01f && fabsf(Vx) > 0.01f)
    {
        float error = (float)(pid_left_total - pid_right_total);

        float P = PID_KP * error;

        pid_integral += error;
        pid_integral  = constrain(pid_integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
        float I = PID_KI * pid_integral;

        float D = PID_KD * (error - pid_prev_error);
        pid_prev_error = error;

        float correction = constrain(P + I + D, -PID_MAX_CORR, PID_MAX_CORR);

        if (correction > 0.0f) {
            left_duty  = constrain(1.0f - correction, 0.0f, 1.0f);
            right_duty = 1.0f;
        } else {
            left_duty  = 1.0f;
            right_duty = constrain(1.0f + correction, 0.0f, 1.0f);
        }
    }
    else
    {
        left_duty  = 1.0f;
        right_duty = 1.0f;
    }

    pwm_counter = (pwm_counter + 1) % PWM_STEPS;

    setMotorPWM(MOTOR1_IN_A, MOTOR1_IN_B, left_rpm,  left_duty);
    setMotorPWM(MOTOR2_IN_A, MOTOR2_IN_B, right_rpm, right_duty);
    setMotorPWM(MOTOR3_IN_A, MOTOR3_IN_B, left_rpm,  left_duty);
    setMotorPWM(MOTOR4_IN_A, MOTOR4_IN_B, right_rpm, right_duty);

    debug_motor_msg.linear.x  = motor_msg.linear.x;
    debug_motor_msg.angular.z = motor_msg.angular.z;
    debug_motor_msg.linear.y  = left_duty;
    debug_motor_msg.linear.z  = right_duty;
}

// -------------------- ENCODER --------------------

void Encoder()
{
    static int32_t prev1 = 0, prev2 = 0, prev3 = 0, prev4 = 0;

    noInterrupts();
    int32_t t1 = motor1_encoder_ticks;
    int32_t t2 = motor2_encoder_ticks;
    int32_t t3 = motor3_encoder_ticks;
    int32_t t4 = motor4_encoder_ticks;
    interrupts();

    int32_t d1 = t1 - prev1;
    int32_t d2 = t2 - prev2;
    int32_t d3 = t3 - prev3;
    int32_t d4 = t4 - prev4;

    prev1 = t1; prev2 = t2; prev3 = t3; prev4 = t4;

    int32_t delta_left  = (d1 + d3);
    int32_t delta_right = (d2 + d4);

    pid_left_total  += delta_left;
    pid_right_total += delta_right;

    // FIX: negated delta_distance — forward motion was counting negative
    float avg_ticks      = (delta_left + delta_right) / 4.0f;
    float revolutions    = avg_ticks / (float)ENCODER_TICKS_PER_REV;
    float delta_distance = revolutions * WHEEL_CIRCUMFERENCE;
    distance_inside_planter -= delta_distance;  // FIX: was +=, now -=

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

    RCCHECK(rclc_subscription_init_best_effort(
        &motor_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/cmd_move"));

    // FIX: reset subscriber now uses Bool instead of Twist
    RCCHECK(rclc_subscription_init_best_effort(
        &reset_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/quin/reset_distance"));

    RCCHECK(rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(20),
        controlCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &twistCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &reset_subscriber, &reset_msg, &resetCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    geometry_msgs__msg__Twist__init(&debug_motor_msg);
    geometry_msgs__msg__Twist__init(&debug_encoder_msg);
    std_msgs__msg__Float32__init(&distance_msg);
    std_msgs__msg__Bool__init(&reset_msg);   // FIX: init Bool instead of Twist

    prev_cmd_time = millis();
    resetPID();

    return true;
}

// -------------------- DESTROY ENTITIES --------------------

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

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
    digitalWrite(MOTOR1_IN_A, LOW); digitalWrite(MOTOR1_IN_B, LOW);
    digitalWrite(MOTOR2_IN_A, LOW); digitalWrite(MOTOR2_IN_B, LOW);
    digitalWrite(MOTOR3_IN_A, LOW); digitalWrite(MOTOR3_IN_B, LOW);
    digitalWrite(MOTOR4_IN_A, LOW); digitalWrite(MOTOR4_IN_B, LOW);
    delay(100);
    ESP.restart();
}