#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

class RobotStatePub{
public:
	RobotStatePub();
	void RobotCmdCallback(const geometry_msgs::Twist::ConstPtr& RobotVel);
private:
	float wheel_seperation_, wheel_radius_;
	ros::Publisher joint_publisher;
	ros::Subscriber sub;
	ros::NodeHandle nh_;
	// const float PI = 3.1415926;
};

RobotStatePub::RobotStatePub():
wheel_radius_(0.1),
wheel_seperation_(0.35)
{
	joint_publisher = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
	sub = nh_.subscribe("robot/cmd_vel", 10, &RobotStatePub::RobotCmdCallback, this);
}

void RobotStatePub::RobotCmdCallback(const geometry_msgs::Twist::ConstPtr& RobotVel){
	static sensor_msgs::JointState robot_state;
	robot_state.name.resize(2);
	robot_state.position.resize(2);
	robot_state.name[0] = "jointL";
	robot_state.name[1] = "jointR";
	robot_state.header.stamp = ros::Time::now();
	float rightWheel = RobotVel->linear.x / wheel_radius_;
	float leftWheel = RobotVel->linear.x / wheel_radius_;
	rightWheel += 2 * RobotVel->angular.z / wheel_seperation_;
	leftWheel += 2 * RobotVel->angular.z / wheel_seperation_;
	joint_publisher.publish(robot_state);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_state_publisher");
	RobotStatePub robot_state_pub;
	ros::spin();
	return 0;
}