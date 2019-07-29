#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void TurtleCallback(const geometry_msgs::Twist::ConstPtr& _twist);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_red_;
  ros::Publisher vel_pub_blue_;
  ros::Subscriber turtle_sub_;
  ros::Subscriber joy_sub_;

};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  //vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
  vel_pub_red_ = nh_.advertise<geometry_msgs::Twist>("red_bot/cmd_vel", 1);
  vel_pub_blue_ = nh_.advertise<geometry_msgs::Twist>("blue_bot/cmd_vel", 1);
  
  turtle_sub_ = nh_.subscribe<geometry_msgs::Twist>("turtle1/cmd_vel", 10, &TeleopTurtle::TurtleCallback, this);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::TurtleCallback(const geometry_msgs::Twist::ConstPtr& _twist)
{
  geometry_msgs::Twist twist;
  twist.angular.z = 0.35*_twist->angular.z;
  twist.linear.x = 0.25*_twist->linear.x;
  vel_pub_red_.publish(twist);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_blue_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
