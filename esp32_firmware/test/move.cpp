#include <Arduino.h>
#include <cmath>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#include "../config/move.h"


// -------- Helpers --------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)


// -------- ROS entities --------

rcl_publisher_t debug_motor_publisher;
geometry_msgs__msg__Twist debug_motor_msg;

rcl_publisher_t debug_encoder_publisher;
geometry_msgs__msg__Twist debug_encoder_msg;

rcl_subscription_t motor_subscriber;        // subscribe /cmd_move
geometry_msgs__msg__Twist motor_msg;

rcl_subscription_t encoder_subscriber;      // subscribe /cmd_encoder
geometry_msgs__msg__Twist encoder_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long current_time = 0;

enum states
{
    WAITING_AGENT,       // 0
    AGENT_AVAILABLE,     // 1
    AGENT_CONNECTED,     // 2
    AGENT_DISCONNECTED   // 3
} state;


// ------ function list ------

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
struct timespec getTime();

void setMotor(int a, int b, int rpm);
void driveDifferential();
void Encoder();


// ------ ENCODER ------

float rpm_motor1 = 0.0;
float rpm_motor2 = 0.0;
float rpm_motor3 = 0.0;
float rpm_motor4 = 0.0;

volatile int32_t motor1_encoder_ticks = 0;
volatile int32_t motor2_encoder_ticks = 0;
volatile int32_t motor3_encoder_ticks = 0;
volatile int32_t motor4_encoder_ticks = 0;

void IRAM_ATTR handleEncoder_MOTOR1() {
  motor1_encoder_ticks += digitalRead(MOTOR1_ENCODER_PIN_B) ? 1 : -1;
}
void IRAM_ATTR handleEncoder_MOTOR2() {
  motor2_encoder_ticks += digitalRead(MOTOR2_ENCODER_PIN_B) ? 1 : -1;
}
void IRAM_ATTR handleEncoder_MOTOR3() {
  motor3_encoder_ticks += digitalRead(MOTOR3_ENCODER_PIN_B) ? 1 : -1;
}
void IRAM_ATTR handleEncoder_MOTOR4() {
  motor4_encoder_ticks += digitalRead(MOTOR4_ENCODER_PIN_B) ? 1 : -1;
}


// ------ main ------

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    set_microros_serial_transports(Serial);     // connect between esp32 and micro-ros agent

    // Setup motors
    pinMode(MOTOR1_IN_A, OUTPUT);   // motor 1
    pinMode(MOTOR1_IN_B, OUTPUT);

    pinMode(MOTOR2_IN_A, OUTPUT);   // motor 2
    pinMode(MOTOR2_IN_B, OUTPUT);

    pinMode(MOTOR3_IN_A, OUTPUT);   // motor 3
    pinMode(MOTOR3_IN_B, OUTPUT);

    pinMode(MOTOR4_IN_A, OUTPUT);   // motor 4
    pinMode(MOTOR4_IN_B, OUTPUT);

    pinMode(MOTOR1_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(MOTOR1_ENCODER_PIN_B, INPUT_PULLUP);

    pinMode(MOTOR2_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(MOTOR2_ENCODER_PIN_B, INPUT_PULLUP);

    pinMode(MOTOR3_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(MOTOR3_ENCODER_PIN_B, INPUT_PULLUP);

    pinMode(MOTOR4_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(MOTOR4_ENCODER_PIN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_PIN_A), handleEncoder_MOTOR1, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCODER_PIN_A), handleEncoder_MOTOR2, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR3_ENCODER_PIN_A), handleEncoder_MOTOR3, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR4_ENCODER_PIN_A), handleEncoder_MOTOR4, RISING);

}


void loop() {
    switch (state) {
        case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(1500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 10)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
        case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) { destroyEntities(); }
        break;
        case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 10)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        }
        break;
        case AGENT_DISCONNECTED:
        destroyEntities();
        state = WAITING_AGENT;
        break;
        default:
        break;
    }
}

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


void twistCallback(const void *msgin)
{   
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    prev_cmd_time = millis();
    motor_msg.linear.x = msg->linear.x;   // forward
    motor_msg.linear.y = msg->linear.y;   // strafe
    motor_msg.angular.z = msg->angular.z; // rotate
}

void twist2Callback(const void *msgin)
{
    prev_cmd_time = millis();
}

bool createEntities()       // create ROS entities 
{
    allocator = rcl_get_default_allocator();    // manage memory of micro-Ros
    geometry_msgs__msg__Twist__init(&debug_motor_msg);
    geometry_msgs__msg__Twist__init(&debug_encoder_msg);

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 77);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    RCCHECK(rclc_node_init_default(&node, "quin_robot_node", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_motor_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/debug/motor"));

    RCCHECK(rclc_subscription_init_best_effort(
        &motor_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/cmd_move"));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_encoder_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/debug/encoder"));

    RCCHECK(rclc_subscription_init_best_effort(
        &encoder_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/cmd_encoder"));

    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(control_timeout), controlCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));    // max handles = 7

    RCCHECK(rclc_executor_add_subscription(
        &executor, &motor_subscriber, &motor_msg, &twistCallback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor, &encoder_subscriber, &encoder_msg, &twist2Callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    syncTime();

    return true;
}


bool destroyEntities()      // destroy ROS entities
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_motor_publisher, &node);
    rcl_publisher_fini(&debug_encoder_publisher, &node);
    rcl_subscription_fini(&motor_subscriber, &node);
    rcl_subscription_fini(&encoder_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}


void Encoder()
{
    static int32_t prev_motor1 = 0, prev_motor2 = 0, prev_motor3 = 0, prev_motor4 = 0;

    int32_t d_motor1 = motor1_encoder_ticks - prev_motor1;
    int32_t d_motor2 = motor2_encoder_ticks - prev_motor2;
    int32_t d_motor3 = motor3_encoder_ticks - prev_motor3;
    int32_t d_motor4 = motor4_encoder_ticks - prev_motor4;

    prev_motor1 = motor1_encoder_ticks;
    prev_motor2 = motor2_encoder_ticks;
    prev_motor3 = motor3_encoder_ticks;
    prev_motor4 = motor4_encoder_ticks;

    float dt = CONTROL_PERIOD_MS / 1000.0f;

    rpm_motor1 = (d_motor1 / (float)ENCODER_TICKS_PER_REV) * (60.0f / dt);
    rpm_motor2 = (d_motor2 / (float)ENCODER_TICKS_PER_REV) * (60.0f / dt);
    rpm_motor3 = (d_motor3 / (float)ENCODER_TICKS_PER_REV) * (60.0f / dt);
    rpm_motor4 = (d_motor4 / (float)ENCODER_TICKS_PER_REV) * (60.0f / dt);

    // fill encoder debug message
    debug_encoder_msg.linear.x  = rpm_motor1;   // tick
    debug_encoder_msg.linear.y  = rpm_motor2;       
    debug_encoder_msg.linear.z  = rpm_motor3;       
    debug_encoder_msg.angular.z = rpm_motor4; 
}


void setMotor(int a, int b, int rpm) {
  if (rpm > 0) {
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
  } else if (rpm < 0) {
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
  } else {
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);
  }
}


void driveDifferential()
{
    float WHEEL_RADIUS = WHEEL_DIAMETER/2;

    // from geometry_msgs/Twist
    float Vx = motor_msg.linear.x;   // m/s
    float Wz = - motor_msg.angular.z;  // rad/s

    // Kinematics for differntial
    float wheel_left = Vx - (Wz * LR_WHEELS_DISTANCE * 0.5f);
    float wheel_right = Vx + (Wz * LR_WHEELS_DISTANCE * 0.5f);

    // m/s to rpm
    float wheel_left_rpm = (wheel_left / (2.0 * M_PI * WHEEL_RADIUS)) * 60.0f; // left
    float wheel_right_rpm = (wheel_right / (2.0 * M_PI * WHEEL_RADIUS)) * 60.0f; // right

    setMotor(MOTOR1_IN_A, MOTOR1_IN_B, wheel_left_rpm);
    setMotor(MOTOR2_IN_A, MOTOR2_IN_B, wheel_right_rpm);
    setMotor(MOTOR3_IN_A, MOTOR3_IN_B, wheel_left_rpm);
    setMotor(MOTOR4_IN_A, MOTOR4_IN_B, wheel_right_rpm);

    debug_motor_msg.linear.y = wheel_left_rpm;   // left
    debug_motor_msg.linear.z = wheel_right_rpm;   // right
}


void publishData()
{
    rcl_publish(&debug_encoder_publisher, &debug_encoder_msg, NULL);

    debug_motor_msg.linear.x = motor_msg.linear.x;   // m/s
    debug_motor_msg.angular.z = motor_msg.angular.z; // rad/s
    rcl_publish(&debug_motor_publisher, &debug_motor_msg, NULL);
}


void syncTime()
{
    // wait for time to be non-zero
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 0;
    do
    {
        ts = getTime();
        delay(10);
    } while (ts.tv_sec == 0 && ts.tv_nsec == 0);

    unsigned long long now_millis = (unsigned long long)ts.tv_sec * 1000 + (unsigned long long)(ts.tv_nsec / 1000000);
    time_offset = now_millis - millis();

    Serial.print("Synchronized time: ");
    Serial.print(ts.tv_sec);
    Serial.print(" sec, ");
    Serial.print(ts.tv_nsec);
    Serial.println(" nsec");
}


struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}


void rclErrorLoop() {

    ESP.restart();
}