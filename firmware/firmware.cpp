// Copyright 2019 Joseph Riesen

// ATmega2560 Firmware for the Tennis Butler Prototype 1

#include <ros.h>
#include <geometry_msgs/Twist.h>

// Minimum number of PCM pulses required for driving
#define L_PULSE_START 0 // Minimum number of PCM pulses required for driving
#define R_PULSE_START 0 // Minimum number of PCM pulses required for driving
#define PULSE_1MPS 100 // PCUL pulse number required for 1.0 m/s
#define PULSE_MAX 128

const int INTAKE_PWM = 128;

// Left motor pins
// const int EncoderAL = 2;
// const int EncoderBL = 3;
const int INA_L = 22;
const int INB_L = 23;
const int DIAGA_L = 24;
const int DIAGB_L = 25;
const int PWM_L = 2; // Timer3
const int CS_L = A0;

// Right motor pins
// const int EncoderAR = 19;
// const int EncoderBR = 18;
const int INA_R = 53;
const int INB_R = 52;
const int DIAGA_R = 51;
const int DIAGB_R = 50;
const int PWM_R = 3; // Timer3
const int CS_R = A1;

// Intake motor pins
const int INA_I = 26;
const int INB_I = 27;
const int DIAGA_I = 28;
const int DIAGB_I = 29;
const int PWM_I = 5; // Timer3
const int CS_I = A2;


ros::NodeHandle nh;

char buffer[128];

int cmdvel_cnt = 0;
int16_t l_motor = 0, r_motor = 0;
bool intake_motor;

void MotorCmdCallback(const geometry_msgs::Twist& msg) {
  cmdvel_cnt = 0;

  intake_motor = (msg.angular.y > 0.0);

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
  if (l_motor > PULSE_MAX)  l_motor = PULSE_MAX;
  if (l_motor < -PULSE_MAX) l_motor = -PULSE_MAX;
  if (r_motor > PULSE_MAX)  r_motor = PULSE_MAX;
  if (r_motor < -PULSE_MAX) r_motor = -PULSE_MAX;

  return;
}

int16_t last_left = 0, last_right = 0;

bool MotorRun(int16_t left, int16_t right, bool intake) {
  if (intake) {
    digitalWrite(INA_I, true);
    digitalWrite(INB_I, false);
    analogWrite(PWM_I, INTAKE_PWM);
  }
  else {
    digitalWrite(INA_I, false);
    digitalWrite(INB_I, false);
    analogWrite(PWM_I, 0);
  }

  if (right > PULSE_MAX || right < -PULSE_MAX || left > PULSE_MAX || left < -PULSE_MAX) {
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
    // Forward on left motor (driver "counter-clockwise", actual motor clockwise)
    // nh.loginfo("Forward on left motor.");
    digitalWrite(INA_L, false);
    digitalWrite(INB_L, true);
    analogWrite(PWM_L, left);
  }
  else if (left < 0) {
    // Reverse left motor direction (driver "clockwise", actual motor counter-clockwise)
    // nh.loginfo("Reverse on left motor.");
    digitalWrite(INA_L, true);
    digitalWrite(INB_L, false);
    analogWrite(PWM_L, -left);
  }
  else {
    // nh.loginfo("Brake left motor.");
    digitalWrite(INA_L, false);
    digitalWrite(INB_L, false);
    analogWrite(PWM_L, 0);
  }

  if (right > 0) {
    // Forward on right motor (driver "counter-clockwise", actual motor clockwise)
    // nh.loginfo("Forward on right motor.");
    digitalWrite(INA_R, false);
    digitalWrite(INB_R, true);
    analogWrite(PWM_R, right);
  }
  else if (right < 0) {
    // Reverse right motor direction (driver "clockwise", actual motor counter-clockwise)
    // nh.loginfo("Reverse on right motor.");
    digitalWrite(INA_R, true);
    digitalWrite(INB_R, false);
    analogWrite(PWM_R, -right);
  }
  else {
    // nh.loginfo("Brake right motor.");
    digitalWrite(INA_R, false);
    digitalWrite(INB_R, false);
    analogWrite(PWM_R, 0);
  }

  return true;
}

bool MotorBrake(void) {
  digitalWrite(INA_L, false);
  digitalWrite(INB_L, false);
  digitalWrite(INA_R, false);
  digitalWrite(INB_R, false);
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
}

ros::Subscriber <geometry_msgs::Twist> sub_cmdvel("alphabot/cmd_vel", MotorCmdCallback);

void setup(void) {
  nh.initNode();
  nh.subscribe(sub_cmdvel);

  const int INA_L = 22;
  const int INB_L = 23;
  const int DIAGA_L = 24;
  const int DIAGB_L = 25;
  const int PWM_L = 2; // Timer1
  const int CS_L = A0;

  pinMode(INA_L, OUTPUT);
  pinMode(INB_L, OUTPUT);
  pinMode(DIAGA_L, INPUT);
  pinMode(DIAGB_L, INPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(CS_L, INPUT);

  pinMode(INA_R, OUTPUT);
  pinMode(INB_R, OUTPUT);
  pinMode(DIAGA_R, INPUT);
  pinMode(DIAGB_R, INPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(CS_R, INPUT);

  pinMode(INA_I, OUTPUT);
  pinMode(INB_I, OUTPUT);
  pinMode(DIAGA_I, INPUT);
  pinMode(DIAGB_I, INPUT);
  pinMode(PWM_I, OUTPUT);
  pinMode(CS_I, INPUT);
}

void loop(void) {
  // Reflect cmd_vel setting
  if (cmdvel_cnt <= 100) {
    // if the cmd_vel instruction is within 1000 ms (100 * 10ms)
    MotorRun(l_motor, r_motor, intake_motor);
    cmdvel_cnt++;
  } else {
    // We have lost contact, apply brakes.
    MotorBrake();
  }
  nh.spinOnce();
  delay(10);
}
