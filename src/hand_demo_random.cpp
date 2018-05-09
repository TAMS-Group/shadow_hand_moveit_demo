#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <random>

#include <iostream>
#include <string>
#include <fstream>

// Run through a set of random hand poses with a right shadow hand.

using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "hand_demo");
	ros::AsyncSpinner spinner(3);
  spinner.start();
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	bool randomize= pnh.param<bool>("random", true);

	ROS_INFO("setting up MGI");
	moveit::planning_interface::MoveGroupInterface mgi("right_hand");

	while(ros::ok()){
    mgi.setRandomTarget();

		bool moved= false;
		if(!(moved= static_cast<bool>(mgi.move()))){
			ROS_WARN_STREAM("Failed to move to state");
		}

		// if something went wrong, we could just continue but abort to debug it
		if(moved) {
			ros::Duration(pnh.param<double>("sleep",2)).sleep();
		}
	}
	return 0;
}
