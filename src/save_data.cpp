#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <iostream>
#include <string>
#include <fstream>

using namespace std;
sensor_msgs::JointState pose_state;
int initialized = 0;

void Two_image_callback(const sensor_msgs::Image::ConstPtr &image_data1, const sensor_msgs::Image::ConstPtr &image_data2) {
	if (initialized)
	{
		static int count = 0;
		count++;
		cout<<"camera count: "<< count << endl;
		int count_=count;

		sensor_msgs::JointState pose = pose_state;

		cv_bridge::CvImagePtr cv_ptr1;
		cv_bridge::CvImagePtr cv_ptr2;
		try
		{
			cv_ptr1 = cv_bridge::toCvCopy(image_data1, sensor_msgs::image_encodings::BGR8);
			cv_ptr2 = cv_bridge::toCvCopy(image_data2, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::imwrite( "/home/hand/v4hn/src/hand_demo/data/handpose/" + to_string( count ) + ".jpg", cv_ptr1->image );
		cv::imwrite( "/home/hand/v4hn/src/hand_demo/data/handpose/" + to_string( ++count ) + ".jpg", cv_ptr2->image );

		ofstream outFile;
		outFile.open("/home/hand/v4hn/src/hand_demo/data/handpose_data.csv",ios::app);
		outFile << to_string( count_ ) << ',' << to_string( count ) << ',' << to_string( pose.position[0]) << ',' << to_string( pose.position[1]) <<','
		<< to_string( pose.position[2]) <<',' << to_string( pose.position[3]) <<',' << to_string( pose.position[4]) <<','
		<< to_string( pose.position[5]) <<',' << to_string( pose.position[6]) <<',' << to_string( pose.position[7]) <<','
		<< to_string( pose.position[8]) <<',' << to_string( pose.position[9]) <<',' << to_string( pose.position[10]) <<','
		<< to_string( pose.position[11]) <<',' << to_string( pose.position[12]) <<',' << to_string( pose.position[13]) <<','
		<< to_string( pose.position[14]) <<',' << to_string( pose.position[15]) <<',' << to_string( pose.position[16]) <<','
		<< to_string( pose.position[17]) <<',' << to_string( pose.position[18]) <<',' << to_string( pose.position[19]) <<','
		<< to_string( pose.position[20]) <<',' << to_string( pose.position[21]) <<',' << to_string( pose.position[22]) <<','
		<< to_string( pose.position[23]) <<  endl;
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "hand_demo_data");
	ros::AsyncSpinner spinner(2);
  spinner.start();
	ros::NodeHandle nh;
	ROS_INFO("setting up MGI");
	moveit::planning_interface::MoveGroupInterface mgi("right_hand");

	message_filters::Subscriber<sensor_msgs::Image> camera1_sub(nh, "/webcam/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> camera2_sub(nh,"/webcam/image_raw2" , 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
	sensor_msgs::Image> SyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), camera1_sub, camera2_sub);
	sync.registerCallback(boost::bind(&Two_image_callback, _1, _2));

	ros::Publisher pose_pub= nh.advertise<sensor_msgs::JointState>("achieved_pose", 1);

	while(ros::ok())
	{
		cout<<"get current pose_state "<<endl;
		pose_state.name = mgi.getJointNames();
		pose_state.position = mgi.getCurrentJointValues();
		pose_state.header.stamp = ros::Time::now();
		initialized = 1;

		pose_pub.publish(pose_state);
	}
	return 0;
}
