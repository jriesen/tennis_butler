#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

typedef enum {
  JOY_AXES_MANUAL_VERTICAL = 1,   // Up/Down Axis stick left
  JOY_AXES_MANUAL_HORIZONTAL = 3, // Left/Right Axis stick right
} JOY_AXES;

typedef enum {
  JOY_BUTTON_A = 0,  // A button on Xbox
} JOY_BUTTON;

#define UPDATE_TS 0.05

class TennisButler {
public:
  TennisButler();
  ~TennisButler();
  void timerCallback(const ros::TimerEvent&);
  void JoyConCallback(const sensor_msgs::Joy &joy_msg);
private:
  ros::Timer timer;
  ros::NodeHandle nh;
  ros::Subscriber joy_sub;
  ros::Publisher twist_pub;
  geometry_msgs::Twist cmd_vel,p_cmd_vel;
  bool last_A;
  bool intake;
};

TennisButler::TennisButler() {
  timer = nh.createTimer(ros::Duration(UPDATE_TS), &TennisButler::timerCallback, this);
  joy_sub = nh.subscribe("joy", 10, &TennisButler::JoyConCallback, this);
  twist_pub = nh.advertise<geometry_msgs::Twist>("alphabot/cmd_vel", 10);
  last_A = false;
  intake = false;
}

TennisButler::~TennisButler(){
}

void TennisButler::JoyConCallback(const sensor_msgs::Joy &joy_msg) {
  if (joy_msg.buttons[JOY_BUTTON_A] && !last_A) {
    intake = !intake;
  }
  last_A = joy_msg.buttons[JOY_BUTTON_A];

  // int boost = (joy_msg.buttons[JOY_BUTTON_MANUALBOOST] == 1) ? 4 : 1;
  cmd_vel.linear.x = joy_msg.axes[JOY_AXES_MANUAL_VERTICAL]; // * 0.25 * boost;
  cmd_vel.angular.z = joy_msg.axes[JOY_AXES_MANUAL_HORIZONTAL]; // * 0.5 * boost;
  cmd_vel.angular.y = intake;
  return;
}

void TennisButler::timerCallback(const ros::TimerEvent&){
  twist_pub.publish(cmd_vel);
  if (p_cmd_vel.linear.x != cmd_vel.linear.x || p_cmd_vel.angular.z != cmd_vel.angular.z ) {
    ROS_INFO("[Manual mode] linear.x: %0.2f, angular.z: %0.2f",cmd_vel.linear.x, cmd_vel.angular.z);
    p_cmd_vel = cmd_vel;
  }
  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "TennisButler");
  TennisButler client;
  ros::spin();
  return 0;
}
