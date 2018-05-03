#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <moveit_msgs/GetRobotStateFromWarehouse.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/collision_detection/collision_matrix.h>

#include <hand_demo/NamedRobotPose.h>

#include <random>

#include <iostream>
#include <string>

// Run through a set of hand poses with a right shadow hand.
//
// Poses are stored in the warehouse or specified in the srdf.
//
// Support small collisions between individual finger links for some gestures.
// Note that this is not inherently safe, but allows for simple contacts
// as long as the planner finds trajectories where the colliding links approach
// the contact from the correct direction

using namespace std;

ros::ServiceClient get_named_state;
bool save_image= false;

bool set_named_target(moveit::planning_interface::MoveGroupInterface& mgi, const string& t){
	moveit_msgs::GetRobotStateFromWarehouse srv;
	srv.request.name= t;
	if( (get_named_state.call(srv) && srv.response.state.joint_state.name.size() > 0) ){
		mgi.setJointValueTarget(srv.response.state.joint_state);
		return true;
	}
	else {
		return mgi.setNamedTarget(t);
	}
}

void tripletcollect_callback30(const sensor_msgs::Image::ConstPtr &image30) {
	static int count = 0;
  	if (save_image)
	{
    		cv_bridge::CvImagePtr cv_ptr30;
    		try
	    	{
	      		cv_ptr30 = cv_bridge::toCvCopy(image30, sensor_msgs::image_encodings::BGR8);
		}
	    	catch (cv_bridge::Exception& e)
	    	{
		       ROS_ERROR("cv_bridge exception: %s", e.what());
		       return;
		}
		cv::imwrite( "/home/hand/v4hn/src/hand_demo/data/30/"  + to_string( count++ )  + ".jpg" , cv_ptr30->image );
               cout<<"count "<<count<<endl;	
        }
        save_image=false;
}

void tripletcollect_callback68(const sensor_msgs::Image::ConstPtr &image68) {
	static int count = 0;
	if (save_image)
	{
	       cv_bridge::CvImagePtr cv_ptr68;
	       try
	       {
			cv_ptr68 = cv_bridge::toCvCopy(image68, sensor_msgs::image_encodings::BGR8);
	       }
	       catch (cv_bridge::Exception& e)
	       {
	    	        ROS_ERROR("cv_bridge exception: %s", e.what());
	                return;
	       }
	       cv::imwrite( "/home/hand/v4hn/src/hand_demo/data/68/"  + to_string( count++ )  + ".jpg", cv_ptr68->image );
        cout<<"count "<< count << endl;        
        }
        save_image=false;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "hand_demo");


	ros::AsyncSpinner spinner(2); // Use 4 threads
        spinner.start();
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	bool randomize= pnh.param<bool>("random", false);

	//publish the jointstate, corresponding name
	ros::Publisher pose_pub= nh.advertise<hand_demo::NamedRobotPose>("achieved_pose", 1);

        //subscribe image
	ros::Subscriber sub1 = nh.subscribe("/camera1/color/image_raw", 1, tripletcollect_callback30);
	ros::Subscriber sub2 = nh.subscribe("/camera2/color/image_raw", 1, tripletcollect_callback68);

	get_named_state= nh.serviceClient<moveit_msgs::GetRobotStateFromWarehouse>("get_robot_state", true);
	ROS_INFO("waiting for warehouse");
	get_named_state.waitForExistence();

	ROS_INFO("setting up MGI and PSI");
	moveit::planning_interface::MoveGroupInterface mgi("right_hand");
	moveit::planning_interface::PlanningSceneInterface psi;

	collision_detection::AllowedCollisionMatrix full_acm;
	{
		moveit_msgs::PlanningScene scene;
		ros::ServiceClient get_planning_scene;
		get_planning_scene= nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene", true);
		ROS_INFO("waiting for planning scene");
		get_planning_scene.waitForExistence();
		moveit_msgs::GetPlanningScene srv;
		srv.request.components.components= moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
		get_planning_scene.call(srv);
		full_acm= srv.response.scene.allowed_collision_matrix;
	}

	typedef struct {
		string name;
		vector<pair<string,string>> allowed_collisions;
	} Target;
	vector<Target> targets {
		//{ "open", {} },
		{ "chinese_number_0", {{"rh_thdistal", "rh_ffdistal"}} },
		{ "chinese_number_1", {{"rh_thdistal", "rh_mfmiddle"}} },
		{ "chinese_number_2", {} },
		{ "chinese_number_3", {{"rh_thdistal", "rh_ffmiddle"}} },
		{ "chinese_number_4", {} },
		{ "chinese_number_5", {} },
		{ "chinese_number_6", {} },
		{ "chinese_number_7", {{"rh_thdistal", "rh_mfdistal"}, {"rh_thdistal", "rh_ffdistal"}} },
		{ "chinese_number_8", {} },
		{ "chinese_number_9", {} },
	};

	size_t current_target= 0;
	while(ros::ok()){
		auto& t= targets[current_target];

		if(!set_named_target(mgi, t.name)){
			ROS_WARN_STREAM("Don't know state '" << t.name << "'. Skipping");
			continue;
		}
		ROS_INFO_STREAM("Going to state " << t.name);

		// allow named collisions
		{
			collision_detection::AllowedCollisionMatrix acm(full_acm);
			for(auto& ac : t.allowed_collisions){
				acm.setEntry(ac.first, ac.second, true);
			}
			moveit_msgs::PlanningScene scene_msg;
			scene_msg.is_diff= true;
			scene_msg.robot_state.is_diff= true;
			acm.getMessage(scene_msg.allowed_collision_matrix);
			psi.applyPlanningScene(scene_msg);
		}

		bool moved= false;
		if(!(moved= static_cast<bool>(mgi.move()))){
			ROS_WARN_STREAM("Failed to move to state '" << t.name << "'");
		}

		// forbid collisions again
		{
			moveit_msgs::PlanningScene scene_msg;
			scene_msg.is_diff= true;
			scene_msg.robot_state.is_diff= true;
			full_acm.getMessage(scene_msg.allowed_collision_matrix);
			psi.applyPlanningScene(scene_msg);
		}

		// something went wrong. We could just continue but abort to debug it
		if(moved) {
			hand_demo::NamedRobotPose pose;
			pose.name=t.name;
			pose.state.name= mgi.getJointNames();
			pose.state.position= mgi.getCurrentJointValues();
			pose.state.header.stamp = ros::Time::now();
			pose_pub.publish(pose);
                        save_image=true;
                        
			ros::Duration(pnh.param<double>("sleep",2)).sleep();
		}
ROS_INFO_STREAM("moved " << moved << " SECONDS");

		if(randomize){
			static std::default_random_engine rnd(42);
			//static std::random_device rnd;
			std::uniform_int_distribution<int> dist(0, targets.size()-1);
			current_target= dist(rnd);
		}
		else {
			current_target= (1+current_target)%targets.size();
			ROS_INFO_STREAM("current target " << current_target << " SECONDS");
                        //if(current_target == 0){
			//	for(int i= 10; i > 0; --i){
			//		ROS_INFO_STREAM("FINISHED ROUND - RESTARTING IN " << i << " SECONDS");
			//		ros::Duration(1.0).sleep();
			//	}
			//}
		}
	}
	return 0;
}
