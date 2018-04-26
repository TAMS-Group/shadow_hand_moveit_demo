#include <ros/ros.h>

#include <moveit_msgs/GetRobotStateFromWarehouse.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/collision_detection/collision_matrix.h>

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

	ros::AsyncSpinner spinner(1);
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
		{ "open", {} },
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
		if(!moved) return 0;

		ros::Duration(pnh.param<double>("sleep",2)).sleep();

		if(randomize){
			static std::default_random_engine rnd(42);
			//static std::random_device rnd;
			std::uniform_int_distribution<int> dist(0, targets.size()-1);
			current_target= dist(rnd);
		}
		else {
			++current_target%targets.size();
		}
	}

	return 0;
}
