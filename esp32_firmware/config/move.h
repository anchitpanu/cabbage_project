#ifndef MOVE_H
#define MOVE_H

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2 
    MOTOR3  MOTOR4  (4WD/DIFFERENTIAL)
         BACK
*/


// Specs of robot
#define MOTOR_MAX_RPM 60
#define MAX_RPM_RATIO 0.85
#define ENCODER_TICKS_PER_REV 600 
#define CONTROL_PERIOD_MS 20
#define WHEEL_DIAMETER 0.085
#define LR_WHEELS_DISTANCE 0.375


// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV true
#define MOTOR3_INV false
#define MOTOR4_INV true


//  Motor Brake
#define MOTOR1_BRAKE true
#define MOTOR2_BRAKE true
#define MOTOR3_BRAKE true
#define MOTOR4_BRAKE true


// Motors Parameters
#define MOTOR1_IN_A 23
#define MOTOR1_IN_B 33

#define MOTOR2_IN_A 18
#define MOTOR2_IN_B 19

#define MOTOR3_IN_A 26
#define MOTOR3_IN_B 27

#define MOTOR4_IN_A 16
#define MOTOR4_IN_B 17


// INVERT ENCODER DIRECTIONS
#define MOTOR1_ENCODER_INV true
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV true 
#define MOTOR4_ENCODER_INV false


// Encoders Parameters
#define MOTOR1_ENCODER_PIN_A 13
#define MOTOR1_ENCODER_PIN_B 32

#define MOTOR2_ENCODER_PIN_A 22
#define MOTOR2_ENCODER_PIN_B 21

#define MOTOR3_ENCODER_PIN_A 25  
#define MOTOR3_ENCODER_PIN_B 14 

#define MOTOR4_ENCODER_PIN_A 4
#define MOTOR4_ENCODER_PIN_B 5


#endif