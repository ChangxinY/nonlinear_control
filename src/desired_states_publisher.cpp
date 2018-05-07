#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <robot/DesiredStates.h>

typedef std::vector<float> one_state;

class DState{
public:
	DState();
	~DState();
private:
	float t, x1, x2, dx1, dx2, ddx1, ddx2, theta;
	float t1, t2;
	std::vector<one_state> states;
	one_state v;
	ros::NodeHandle nh;
	ros::Publisher state_pub;
	std::vector<one_state>::size_type i;
	robot::DesiredStates xd;
	void readfile();
};

DState::DState()
{
	readfile();
	state_pub = nh.advertise<robot::DesiredStates>("/desired_states", 100);
	i = 1;
	t1 = states[i-1][0];
	t2 = states[i][0];
	ros::Rate r(100);
	while(i<states.size()){
		v = states[i-1];
		xd.t = v[0];
		xd.x1 = v[1] * 0.01;
		xd.x2 = v[2] * 0.01;
		xd.dx1 = v[3] * 0.01;
		xd.dx2 = v[4] * 0.01;
		xd.ddx1 = v[5] * 0.01;
		xd.ddx2 = v[6] * 0.01;
		xd.theta = v[7];
		ros::Time begin = ros::Time::now();
		ros::Time end = ros::Time::now();
		ros::Duration diff = end - begin;
		while(ros::ok() && diff.toSec()<=t2-t1){
			end = ros::Time::now();
			xd.header.stamp = end;
			state_pub.publish(xd);
			diff = end - begin;
			r.sleep();
		}
		++i;
		t1 = states[i-1][0];
		t2 = states[i][0];
	}
}

DState::~DState(){
	states.clear();
}

void DState::readfile(){
	std::ifstream infile("/home/changxin/nonlinear_ws/src/robot/src/final_trajectory_2.txt");
	if(!infile){
		ROS_INFO("data not read");
	}
	while(infile >> t >> x1 >> x2 >> dx1 >> dx2 >> ddx1 >> ddx2 >> theta){
		v.push_back(t);
		v.push_back(x1);
		v.push_back(x2);
		v.push_back(dx1);
		v.push_back(dx2);
		v.push_back(ddx1);
		v.push_back(ddx2);
		v.push_back(theta);
		states.push_back(v);
		v.clear();
	}
	infile.close();
	return;
}

int main(int argc, char**argv){
	// vector<vector<float>> a;
	ros::init(argc, argv, "xd_publisher");
	DState ddstate;
	ros::spin();
	return 0;
}	