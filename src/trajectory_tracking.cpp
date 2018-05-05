#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <robot/DesiredStates.h>
#include <geometry_msgs/Twist.h>
#include <iostream>


class TrajecTracking{
public:
	TrajecTracking();
private:
	void trajecingCB(const nav_msgs::Odometry::ConstPtr& odome);
	void getDesiredStatesCB(const robot::DesiredStates::ConstPtr& desiredSt);
	ros::NodeHandle nh;
	ros::Publisher vel_pub;
	ros::Subscriber odeom_sub;
	ros::Subscriber desired_state_sub;
	float x1_d, x2_d, dx1_d, dx2_d, ddx1_d, ddx2_d;
	float pre_dx1, pre_dx2;
	bool new_state;
	float kp, kd;
	float u1;
};

TrajecTracking::TrajecTracking():
new_state(false),
pre_dx1(0),
pre_dx2(0),
kp(10),
kd(1),
u1(0.1)
{
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	desired_state_sub = nh.subscribe<robot::DesiredStates>("/desired_states", 10, &TrajecTracking::getDesiredStatesCB, this);
	odeom_sub= nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &TrajecTracking::trajecingCB, this);

}

void TrajecTracking::trajecingCB(const nav_msgs::Odometry::ConstPtr& odome){
	if(new_state){
		float x1, x2, dx1, dx2, ddx1, ddx2;
		x1 = odome->pose.pose.position.x;
		x2 = odome->pose.pose.position.y;
		dx1 = odome->twist.twist.linear.x;
		dx2 = odome->twist.twist.linear.y;
		ddx1 = (dx1 - pre_dx1) / 0.1;
		ddx2 = (dx2 - pre_dx2) / 0.1;
		pre_dx1 = dx1;
		pre_dx2 = dx2;
		float v1 = ddx1_d - kp*(x1 - x1_d) - kd*(dx1 - dx1_d);
		float v2 = ddx2_d - kp*(x2 - x2_d) - kd*(dx2 - dx2_d);
		float theta = atan2(dx2, dx1);
		float v_input1 = cos(theta) * v1 + sin(theta) * v2;
		float v_input2 = cos(theta) * v2 - sin(theta) * v1;
		u1 += v_input1 * 0.1;
		float u2 = v_input2 / u1;
		geometry_msgs::Twist cmd;
		cmd.linear.x = u1;
		cmd.angular.z = u2;
		vel_pub.publish(cmd);
		std::cout << "ddx1 " << ddx1 << " ddx2 " << ddx2 << std::endl;
		//<< " x1_d " << x1_d << " x2_d" << x2_d << std::endl;
		std::cout << std::endl;
		std::cout << "x1 " << x1 << " x2 " << x2 << " x1_d " << x1_d << " x2_d" << x2_d << std::endl;
		std::cout << std::endl;
		std::cout << "linear velocity " << u1 << " angular velocity " << u2 << std::endl;
		std::cout << std::endl;
	}
}

void TrajecTracking::getDesiredStatesCB(const robot::DesiredStates::ConstPtr& desiredSt){
	x1_d = desiredSt->x1;
	x2_d = desiredSt->x2;
	dx1_d = desiredSt->dx1;
	dx2_d = desiredSt->dx2;
	ddx1_d = desiredSt->ddx1;
	ddx2_d = desiredSt->ddx2;
	new_state = true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "trajec_tracking");
	TrajecTracking tt;
	ros::spin();
	return 0;
}
