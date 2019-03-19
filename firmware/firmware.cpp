// Copyright 2019 Joseph Riesen

// ATmega2560 Firmware for the Tennis Butler Prototype 1

#include <ros.h>
#include <geometry_msgs/Twist.h>

// Minimum number of PCM pulses required for driving
#define L_PULSE_START 130 // Minimum number of PCM pulses required for driving
#define R_PULSE_START 130 // Minimum number of PCM pulses required for driving
#define PULSE_1MPS 120 // PCUL pulse number required for 1.0 m/s

// Left motor pins
// const int EncoderAL = 2;
// const int EncoderBL = 3;
const int ENL = 22;
const int ENBL = 23;
const int PWM2L = 11; // Timer1
const int PWM1L = 12; // Timer1
const int OCML = A0;
const int DIAGL = 24;

// Right motor pins
// const int EncoderAR = 19;
// const int EncoderBR = 18;
const int ENR = 25;
const int ENBR = 26;
const int PWM2R = 2; // Timer3
const int PWM1R = 3; // Timer3
const int OCMR = A1;
const int DIAGR = 27;

ros::NodeHandle nh;

char buffer[128];

int cmdvel_cnt = 0;
int16_t l_motor = 0, r_motor = 0;

void MotorCmdCallback(const geometry_msgs::Twist& msg) {
  cmdvel_cnt = 0;

  // Setting direction of travel
  if (msg.linear.x > 0.0) {
    l_motor = (int) (L_PULSE_START + PULSE_1MPS * msg.linear.x);
    r_motor = (int) (R_PULSE_START + PULSE_1MPS * msg.linear.x);
  } else if (msg.linear.x < 0.0) {
    l_motor = (int) (-1 * L_PULSE_START + PULSE_1MPS * msg.linear.x);
    r_motor = (int) (-1 * R_PULSE_START + PULSE_1MPS * msg.linear.x);
  } else {
    l_motor = 0;
    r_motor = 0;
  }

  // Angle direction setting
  if (msg.angular.z > 0.0) {
    if (msg.linear.x == 0.0) {
      l_motor = (int) (-1 * L_PULSE_START - PULSE_1MPS * msg.angular.z);
      r_motor = (int) (R_PULSE_START + PULSE_1MPS * msg.angular.z);
    } else if (msg.linear.x> 0.0) {
      r_motor += PULSE_1MPS * msg.angular.z;
      l_motor -= PULSE_1MPS * msg.angular.z * 0.5;
    } else {
      r_motor -= PULSE_1MPS * msg.angular.z;
      l_motor += PULSE_1MPS * msg.angular.z * 0.5;
    }
  } else if (msg.angular.z < 0.0) {
    if (msg.linear.x == 0.0) {
      l_motor = (int) (L_PULSE_START + PULSE_1MPS * msg.angular.z * -1);
      r_motor = (int) (-1 * R_PULSE_START - PULSE_1MPS * msg.angular.z * -1);
    } else if (msg.linear.x> 0.0) {
      l_motor += PULSE_1MPS * msg.angular.z * -1;
      r_motor -= PULSE_1MPS * msg.angular.z * 0.5 * -1;
    } else {
      l_motor -= PULSE_1MPS * msg.angular.z * -1;
      r_motor += PULSE_1MPS * msg.angular.z * 0.5 * -1;
    }
  }

  // Clamp values
  if (l_motor > 255)  l_motor = 255;
  if (l_motor < -255) l_motor = -255;
  if (r_motor > 255)  r_motor = 255;
  if (r_motor < -255) r_motor = -255;

  return;
}

int16_t last_left = 0, last_right = 0;

bool MotorRun(int16_t left, int16_t right) {
  // Disengage braking
  digitalWrite(ENBR, false);
  digitalWrite(ENBL, false);

  if (right > 255 || right < -255 || left > 255 || left < -255) {
    nh.logwarn("Motor speed out-of-range!");
    return false;
  }

  if (left != last_left || right != last_right) {
    sprintf(buffer, "Left motor: %d, Right motor: %d", left, right);
    nh.loginfo(buffer);
    last_left = left;
    last_right = right;
  }

  if (left > 0) {
    // Forward on left motor
    // nh.loginfo("Forward on left motor.");
    digitalWrite(ENL, true);
    digitalWrite(ENBL, false);
    analogWrite(PWM1L, left);
    analogWrite(PWM2L, 0);
  }
  else if (left < 0) {
    // Reverse left motor direction
    // nh.loginfo("Reverse on left motor.");
    digitalWrite(ENL, true);
    digitalWrite(ENBL, false);
    analogWrite(PWM1L, 0);
    analogWrite(PWM2L, -left);
  }
  else {
    // nh.loginfo("Brake left motor.");
    digitalWrite(ENL, true);
    digitalWrite(ENBL, false);
    analogWrite(PWM1L, 0);
    analogWrite(PWM2L, 0);
  }

  if (right > 0) {
    // nh.loginfo("Forward on right motor.");
    digitalWrite(ENR, true);
    digitalWrite(ENBR, false);
    analogWrite(PWM1R, right);
    analogWrite(PWM2R, 0);
  }
  else if (right < 0) {
    // Reverse right motor direction
    // nh.loginfo("Reverse on right motor.");
    digitalWrite(ENR, true);
    digitalWrite(ENBR, false);
    analogWrite(PWM1R, 0);
    analogWrite(PWM2R, -right);
  }
  else {
    // nh.loginfo("Brake right motor.");
    digitalWrite(ENR, true);
    digitalWrite(ENBR, false);
    analogWrite(PWM1R, 0);
    analogWrite(PWM2R, 0);
  }

  return true;
}

bool MotorBrake(void) {
  digitalWrite(ENL, true);
  digitalWrite(ENBL, false);
  digitalWrite(ENR, true);
  digitalWrite(ENBR, false);
  analogWrite(PWM1L, 0);
  analogWrite(PWM2L, 0);
  analogWrite(PWM1R, 0);
  analogWrite(PWM2R, 0);
}

ros::Subscriber <geometry_msgs::Twist> sub_cmdvel("alphabot/cmd_vel", MotorCmdCallback);

void setup(void) {
  nh.initNode();
  nh.subscribe(sub_cmdvel);

  pinMode(ENL, OUTPUT);
  pinMode(ENBL, OUTPUT);
  // pinMode(PWM1L, OUTPUT);
  // pinMode(PWM2L, OUTPUT);
  pinMode(OCML, INPUT);
  pinMode(DIAGL, INPUT_PULLUP);

  pinMode(ENR, OUTPUT);
  pinMode(ENBR, OUTPUT);
  // pinMode(PWM1R, OUTPUT);
  // pinMode(PWM2R, OUTPUT);
  pinMode(OCMR, INPUT);
  pinMode(DIAGR, INPUT_PULLUP);
}

void loop(void) {
  // Reflect cmd_vel setting
  if (cmdvel_cnt <= 100) {
    // if the cmd_vel instruction is within 1000 ms (100 * 10ms)
    MotorRun(l_motor, r_motor);
    cmdvel_cnt++;
  } else {
    // We have lost contact, apply brakes.
    MotorBrake();
  }
  nh.spinOnce();
  delay(10);
}
