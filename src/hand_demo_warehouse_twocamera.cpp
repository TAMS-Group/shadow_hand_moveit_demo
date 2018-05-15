#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <moveit_msgs/GetRobotStateFromWarehouse.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/collision_detection/collision_matrix.h>

#include <hand_demo/NamedRobotPose.h>

#include <random>

#include <iostream>
#include <string>
#include <fstream>

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
sensor_msgs::JointState pose_state;
bool planned = 0;
bool moved= false;

void Two_image_callback(const sensor_msgs::Image::ConstPtr &image_data1, const sensor_msgs::Image::ConstPtr &image_data2) {
	if (planned && moved)
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
		ros::Duration(1).sleep();
	}
}


void get_current_states(const sensor_msgs::JointState &current_state){
	// cout<<"get current pose_state "<<endl;
	pose_state=current_state;
	pose_state.header.stamp = ros::Time::now();
}


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


int main(int argc, char** argv){
	ros::init(argc, argv, "hand_demo_warehouse_twocamera");
	ros::AsyncSpinner spinner(3);
	spinner.start();
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	bool randomize= pnh.param<bool>("random", false);

	get_named_state= nh.serviceClient<moveit_msgs::GetRobotStateFromWarehouse>("get_robot_state", true);
	ROS_INFO("waiting for warehouse");
	get_named_state.waitForExistence();

	ROS_INFO("setting up MGI and PSI");
	moveit::planning_interface::MoveGroupInterface mgi("right_hand");
	moveit::planning_interface::PlanningSceneInterface psi;
	moveit::planning_interface::MoveGroupInterface::Plan shadow_plan;

	ros::Subscriber state_sub = nh.subscribe("/joint_states", 1, get_current_states);

	message_filters::Subscriber<sensor_msgs::Image> camera1_sub(nh, "/webcam/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> camera2_sub(nh,"/webcam/image_raw2" , 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
	sensor_msgs::Image> SyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), camera1_sub, camera2_sub);
	sync.registerCallback(boost::bind(&Two_image_callback, _1, _2));

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr shadow_model = robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(shadow_model);
	robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
	const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("right_hand");

	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	vector<pair<string,string>> collision_pairs;

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

	vector<string> targets {
		"open",
		"chinese_number_0",
		"chinese_number_1",
		"chinese_number_2",
		"chinese_number_3",
		"chinese_number_4",
		"chinese_number_5",
		"chinese_number_6",
		"chinese_number_7",
		"chinese_number_8",
		"chinese_number_9",
		"grasp_0_can",
		"grasp_0_thincan",
		"grasp_0_bigcan",
		"grasp_1_apple",
		"grasp_2_ffup",
		"grasp_3_smallcube",
		"grasp_3_smallcylinder",
		"grasp_4_pen",
		"grasp_4_pen1",
		"grasp_parallel",
		"grasp_parallel2",
		"grasp_parallel3",
		"grasp_smallcylinder2",
		"grasp_thinobjs",
	};

	size_t current_target= 0;
	while(ros::ok()){
		auto& t= targets[current_target];

		if(!set_named_target(mgi, t)){
			ROS_WARN_STREAM("Don't know state '" << t << "'. Skipping");
			continue;
		}
		ROS_INFO_STREAM("Going to state " << t);

		collision_request.contacts = true;
		collision_request.max_contacts = 1000;
		collision_result.clear();

		// get self collision results;
		current_state = mgi.getJointValueTarget();
		planning_scene.checkSelfCollision(collision_request, collision_result);
		collision_detection::CollisionResult::ContactMap::const_iterator it;
		collision_pairs.clear();
		for(it = collision_result.contacts.begin();	it != collision_result.contacts.end(); ++it)
		{
			collision_pairs.push_back(std::make_pair (it->first.first.c_str(), it->first.second.c_str()));
			ROS_WARN("Collision between: %s and %s, need to allow these collisions", it->first.first.c_str(), it->first.second.c_str());
		}

		// allow named collisions
		{
			collision_detection::AllowedCollisionMatrix acm(full_acm);
			for(auto& ac : collision_pairs){
				acm.setEntry(ac.first, ac.second, true);
			}
			moveit_msgs::PlanningScene scene_msg;
			scene_msg.is_diff= true;
			scene_msg.robot_state.is_diff= true;
			acm.getMessage(scene_msg.allowed_collision_matrix);
			psi.applyPlanningScene(scene_msg);
		}

		planned = false;
		if (!(planned= static_cast<bool>(mgi.plan(shadow_plan))))
		{
			ROS_WARN_STREAM("Failed to plan state '" << t<< "'");
		}

		if(!(moved= static_cast<bool>(mgi.execute(shadow_plan)))){
			ROS_WARN_STREAM("Failed to execute state '" << t<< "'");
		}

		// forbid collisions again
		{
			moveit_msgs::PlanningScene scene_msg;
			scene_msg.is_diff= true;
			scene_msg.robot_state.is_diff= true;
			full_acm.getMessage(scene_msg.allowed_collision_matrix);
			psi.applyPlanningScene(scene_msg);
		}

		if(randomize){
			static std::default_random_engine rnd(time(nullptr));
			//static std::random_device rnd;
			std::uniform_int_distribution<int> dist(0, targets.size()-1);
			current_target= dist(rnd);
		}
		else {
			current_target= (1+current_target)%targets.size();
		}
	}
	return 0;
}
