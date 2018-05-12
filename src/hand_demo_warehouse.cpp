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
	ros::init(argc, argv, "hand_demo");
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

		bool moved= false;
		if(!(moved= static_cast<bool>(mgi.move()))){
			ROS_WARN_STREAM("Failed to move to state '" << t<< "'");
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
			ros::Duration(pnh.param<double>("sleep",2)).sleep();
		}

		if(randomize){
			static std::default_random_engine rnd(42);
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
