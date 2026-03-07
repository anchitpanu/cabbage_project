#include <Arduino.h>
#include <cmath>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <ESP32Encoder.h>

#include "../config/base_move.h" 


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

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

int32_t lastPosition = 0;
unsigned long lastTime = 0;

float motor1RPM = 0;
float motor2RPM = 0;
float motor3RPM = 0;
float motor4RPM = 0;

// --- Distance/odometry state (meters/radians) ---
double dist1 = 0.0, dist2 = 0.0, dist3 = 0.0, dist4 = 0.0;  // per-wheel accumulated distance
double dist_left = 0.0, dist_right = 0.0, dist_avg = 0.0;   // left/right/average distance
double theta_sum = 0.0;                                     // accumulated rotation (rad)

// --- Encoder counting mode factor ---
// Using attachHalfQuad() → counts x2
// (if using attachFullQuad() instead, set this to 4)
const int ENCODER_QUAD = 2;


// ------ function list ------

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
struct timespec getTime();

void Move();
void Encoder();

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

    // Motor 1
    if (MOTOR1_ENCODER_INV)
        encoder1.attachHalfQuad(MOTOR1_ENCODER_PIN_B, MOTOR1_ENCODER_PIN_A);
    else
        encoder1.attachHalfQuad(MOTOR1_ENCODER_PIN_A, MOTOR1_ENCODER_PIN_B);

    // Motor 2
    if (MOTOR2_ENCODER_INV)
        encoder2.attachHalfQuad(MOTOR2_ENCODER_PIN_B, MOTOR2_ENCODER_PIN_A);
    else
        encoder2.attachHalfQuad(MOTOR2_ENCODER_PIN_A, MOTOR2_ENCODER_PIN_B);

    // Motor 3
    if (MOTOR3_ENCODER_INV)
        encoder3.attachHalfQuad(MOTOR3_ENCODER_PIN_B, MOTOR3_ENCODER_PIN_A);
    else
        encoder3.attachHalfQuad(MOTOR3_ENCODER_PIN_A, MOTOR3_ENCODER_PIN_B);

    // Motor 4
    if (MOTOR4_ENCODER_INV)
        encoder4.attachHalfQuad(MOTOR4_ENCODER_PIN_B, MOTOR4_ENCODER_PIN_A);
    else
        encoder4.attachHalfQuad(MOTOR4_ENCODER_PIN_A, MOTOR4_ENCODER_PIN_B);

    encoder1.clearCount();
    encoder2.clearCount();
    encoder3.clearCount();
    encoder4.clearCount();
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

// ------ functions ------

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        Move();
        Encoder();
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
        &debug_encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/quin/debug/encoder"));

    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(control_timeout), controlCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));    // max handles = 7

    RCCHECK(rclc_executor_add_subscription(
        &executor, &motor_subscriber, &motor_msg, &twistCallback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor, &motor_subscriber, &motor_msg, &twist2Callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    syncTime();

    return true;
}

bool destroyEntities()      // destroy ROS entities
{
    // rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    // (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_motor_publisher, &node);
    rcl_publisher_fini(&debug_encoder_publisher, &node);
    rcl_subscription_fini(&motor_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void Move()
{
    float WHEEL_RADIUS = WHEEL_DIAMETER/2;

    // from geometry_msgs/Twist
    float Vx = motor_msg.linear.x;   // m/s
    float Wz = -motor_msg.angular.z;  // rad/s

    // Kinematics for differntial
    float wheel_left = Vx - (Wz * LR_WHEELS_DISTANCE * 0.5f);
    float wheel_right = Vx + (Wz * LR_WHEELS_DISTANCE * 0.5f);

    // m/s to rpm
    float wheel1_rpm = (wheel_left / (2.0 * M_PI * WHEEL_RADIUS)) * 60.0f; // front left
    float wheel2_rpm = (wheel_right / (2.0 * M_PI * WHEEL_RADIUS)) * 60.0f; // front right
    float wheel3_rpm = (wheel_left / (2.0 * M_PI * WHEEL_RADIUS)) * 60.0f; // rear left
    float wheel4_rpm = (wheel_right / (2.0 * M_PI * WHEEL_RADIUS)) * 60.0f; // rear right

    // Clamp RPM to max allowed
    float max_rpm_allowed = MOTOR_MAX_RPM * MAX_RPM_RATIO;
    wheel1_rpm = constrain(wheel1_rpm, -max_rpm_allowed, max_rpm_allowed);
    wheel2_rpm = constrain(wheel2_rpm, -max_rpm_allowed, max_rpm_allowed);
    wheel3_rpm = constrain(wheel3_rpm, -max_rpm_allowed, max_rpm_allowed);
    wheel4_rpm = constrain(wheel4_rpm, -max_rpm_allowed, max_rpm_allowed);

    // Convert RPM to PWM duty cycle
    int pwm1 = (int)((fabs(wheel1_rpm) / MOTOR_MAX_RPM) * PWM_Max);
    int pwm2 = (int)((fabs(wheel2_rpm) / MOTOR_MAX_RPM) * PWM_Max);
    int pwm3 = (int)((fabs(wheel3_rpm) / MOTOR_MAX_RPM) * PWM_Max);
    int pwm4 = (int)((fabs(wheel4_rpm) / MOTOR_MAX_RPM) * PWM_Max);

    // Set motor directions and apply PWM
    if (wheel1_rpm > 0)     // wheel 1
    {
        digitalWrite(MOTOR1_IN_A, HIGH ^ MOTOR1_INV);
        digitalWrite(MOTOR1_IN_B, LOW ^ MOTOR1_INV);
        ledcWrite(0, pwm1);
    }
    else if (wheel1_rpm < 0)
    {
        digitalWrite(MOTOR1_IN_A, LOW ^ MOTOR1_INV);
        digitalWrite(MOTOR1_IN_B, HIGH ^ MOTOR1_INV);
        ledcWrite(0, pwm1);
    }
    else
    {
        digitalWrite(MOTOR1_IN_A, LOW);
        digitalWrite(MOTOR1_IN_B, LOW);
        ledcWrite(0, 0);
    }

    if (wheel2_rpm > 0)     // wheel 2
    {
        digitalWrite(MOTOR2_IN_A, HIGH ^ MOTOR2_INV);
        digitalWrite(MOTOR2_IN_B, LOW ^ MOTOR2_INV);
        ledcWrite(0, pwm2);
    }
    else if (wheel2_rpm < 0)
    {
        digitalWrite(MOTOR2_IN_A, LOW ^ MOTOR2_INV);
        digitalWrite(MOTOR2_IN_B, HIGH ^ MOTOR2_INV);
        ledcWrite(0, pwm2);
    }
    else
    {
        digitalWrite(MOTOR2_IN_A, LOW);
        digitalWrite(MOTOR2_IN_B, LOW);
        ledcWrite(0, 0);
    }
    
    if (wheel3_rpm > 0)     // wheel 3
    {
        digitalWrite(MOTOR3_IN_A, HIGH ^ MOTOR3_INV);
        digitalWrite(MOTOR3_IN_B, LOW ^ MOTOR3_INV);
        ledcWrite(0, pwm3);
    }
    else if (wheel3_rpm < 0)
    {
        digitalWrite(MOTOR3_IN_A, LOW ^ MOTOR3_INV);
        digitalWrite(MOTOR3_IN_B, HIGH ^ MOTOR3_INV);
        ledcWrite(0, pwm3);
    }
    else
    {
        digitalWrite(MOTOR3_IN_A, LOW);
        digitalWrite(MOTOR3_IN_B, LOW);
        ledcWrite(0, 0);
    }

    if (wheel4_rpm > 0)     // wheel 4
    {
        digitalWrite(MOTOR4_IN_A, HIGH ^ MOTOR4_INV);
        digitalWrite(MOTOR4_IN_B, LOW ^ MOTOR4_INV);
        ledcWrite(0, pwm4);
    }
    else if (wheel4_rpm < 0)
    {
        digitalWrite(MOTOR4_IN_A, LOW ^ MOTOR4_INV);
        digitalWrite(MOTOR4_IN_B, HIGH ^ MOTOR4_INV);
        ledcWrite(0, pwm4);
    }
    else
    {
        digitalWrite(MOTOR4_IN_A, LOW);
        digitalWrite(MOTOR4_IN_B, LOW);
        ledcWrite(0, 0);
    }

    debug_motor_msg.linear.y = wheel1_rpm;   // front left
    debug_motor_msg.linear.z = wheel2_rpm;   // front right
    debug_motor_msg.angular.x = wheel3_rpm;  // rear left
    debug_motor_msg.angular.y = wheel4_rpm;  // rear right


}

void Encoder() {
    // track time (not needed for distance, but kept for consistency)
    // current_time = millis();
    // unsigned long time_diff = (current_time - lastTime);
    // lastTime = current_time;

    // read incremental counts
    long c1 = encoder1.getCount();
    long c2 = encoder2.getCount();
    long c3 = encoder3.getCount();
    long c4 = encoder4.getCount();

    // pulses per wheel revolution (encoder + counting mode + gear ratio)
    const double counts_per_wheel_rev = (double)PULSES_PER_REVOLUTION * (double)ENCODER_QUAD * (double)GEAR_RATIO;
    const double wheel_circumference = (double)WHEEL_DIAMETER * M_PI;  // meters

    // convert to distance increment (m)
    const double d1 = ((double)c1 / counts_per_wheel_rev) * wheel_circumference;
    const double d2 = ((double)c2 / counts_per_wheel_rev) * wheel_circumference;
    const double d3 = ((double)c3 / counts_per_wheel_rev) * wheel_circumference;
    const double d4 = ((double)c4 / counts_per_wheel_rev) * wheel_circumference;

    // accumulate wheel distances
    dist1 += d1;  dist2 += d2;  dist3 += d3;  dist4 += d4;

    // left/right increments (average of front+rear per side)
    const double dL = 0.5 * (d1 + d3);  // left  = wheel1 (front-left) + wheel3 (rear-left)
    const double dR = 0.5 * (d2 + d4);  // right = wheel2 (front-right) + wheel4 (rear-right)

    // accumulate chassis odometry
    dist_left  += dL;   
    dist_right += dR;   
    dist_avg    = 0.5 * (dist_left + dist_right);

    // accumulate orientation (rad)
    theta_sum += (dR - dL) / (double)LR_WHEELS_DISTANCE;

    // reset encoder counters to measure increments again
    encoder1.clearCount();
    encoder2.clearCount();
    encoder3.clearCount();
    encoder4.clearCount();

        // Prepare message fields (will publish in publishData)
    debug_encoder_msg.linear.x  = (float)dist_left;                    // left distance [m]
    debug_encoder_msg.linear.y  = (float)dist_right;                   // right distance [m]
    debug_encoder_msg.linear.z  = (float)((dist_left+ dist_right) * 0.5); // average [m]
    debug_encoder_msg.angular.z = (float)theta_sum;                     // yaw [rad]
}

void publishData()
{
    debug_motor_msg.linear.x = motor_msg.linear.x;   // m/s
    debug_motor_msg.angular.z = motor_msg.angular.z; // rad/s
    rcl_publish(&debug_motor_publisher, &debug_motor_msg, NULL);

    float leftRPM  = (motor1RPM + motor3RPM) / 2.0;
    float rightRPM = (motor2RPM + motor4RPM) / 2.0;

    float wheel_circumference = WHEEL_DIAMETER * PI;
    float leftLinear  = leftRPM  * wheel_circumference / 60.0;
    float rightLinear = rightRPM * wheel_circumference / 60.0;

    debug_encoder_msg.linear.x  = (leftLinear + rightLinear) / 2.0;                     // linear velocity in m/s
    debug_encoder_msg.angular.z = (rightLinear - leftLinear) / LR_WHEELS_DISTANCE;      // angular velocity in rad/s
    struct timespec time_stamp = getTime();
    rcl_publish(&debug_encoder_publisher, &debug_encoder_msg, NULL);
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




