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

#include <random>

#include <iostream>
#include <string>
#include <fstream>

// Run through a set of hand poses with a right shadow hand.
//
// Poses are stored in the warehouse or specified in the srdf.
//
// use one camera to sve anchor positive and negative.
// margin range

using namespace std;

ros::ServiceClient get_named_state;
sensor_msgs::JointState pose_state;
bool planned = false;
bool moved = false;
bool initialized = false;

void One_camera_callback(const sensor_msgs::Image::ConstPtr &image_data) {
	// if (!planned && initialized)
	// {
	// 	//create new folfer
	// 	// image folder = "/home/hand/v4hn/src/hand_demo/data/handpose/";
	// 	// image_folder = main_folder + ""
	// }
	static int count = 0;
	if (planned)
	{
		// after planned, waiting robot move
		if (!initialized)
		{
			ros::Duration(0.5).sleep();
			initialized = true;
			return;
		}

		sensor_msgs::JointState pose = pose_state;
		cv_bridge::CvImagePtr cv_ptr;

		try
		{
			cv_ptr = cv_bridge::toCvCopy(image_data, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		count++;
		cout<<"image count: "<< count << endl;

		cv::imwrite( "/home/hand/v4hn/src/hand_demo/data/handpose14/" + to_string( count ) + ".jpg", cv_ptr->image );

		ofstream outFile;
		outFile.open("/home/hand/v4hn/src/hand_demo/data/handpose_data/handpose_data14.csv",ios::app);
		outFile << to_string( count ) << ',' << to_string( pose.position[0]) << ',' << to_string( pose.position[1]) <<','
		<< to_string( pose.position[2]) <<',' << to_string( pose.position[3]) <<',' << to_string( pose.position[4]) <<','
		<< to_string( pose.position[5]) <<',' << to_string( pose.position[6]) <<',' << to_string( pose.position[7]) <<','
		<< to_string( pose.position[8]) <<',' << to_string( pose.position[9]) <<',' << to_string( pose.position[10]) <<','
		<< to_string( pose.position[11]) <<',' << to_string( pose.position[12]) <<',' << to_string( pose.position[13]) <<','
		<< to_string( pose.position[14]) <<',' << to_string( pose.position[15]) <<',' << to_string( pose.position[16]) <<','
		<< to_string( pose.position[17]) <<',' << to_string( pose.position[18]) <<',' << to_string( pose.position[19]) <<','
		<< to_string( pose.position[20]) <<',' << to_string( pose.position[21]) <<',' << to_string( pose.position[22]) <<','
		<< to_string( pose.position[23]) <<  endl;
	}

	//record the start "point" of a new action
	if (!planned && initialized)
	{
		initialized = false;
		ofstream outFile;
		outFile.open("/home/hand/v4hn/src/hand_demo/data/handpose_changepoint/handpose_changepoint14.csv", ios::app);
		outFile << to_string( count + 1 ) << endl;
	}
	ros::Duration(0.05).sleep();
}


void get_current_states(const sensor_msgs::JointState &current_state){
	//cout<<"get current pose_state "<<endl;
	pose_state=current_state;
	pose_state.header.stamp = ros::Time::now();
	// initialized = 1;
}


bool set_named_target(moveit::planning_interface::MoveGroupInterface& mgi, const string& t, double w1, double w2){
	moveit_msgs::GetRobotStateFromWarehouse srv;
	srv.request.name= t;
	if( (get_named_state.call(srv) && srv.response.state.joint_state.name.size() > 0) ){
		sensor_msgs::JointState state;
		state = srv.response.state.joint_state;
		state.position[23] = w1;
		state.position[22] = w2;
		mgi.setJointValueTarget(state);
		// mgi.setJointValueTarget(srv.response.state.joint_state);
		return true;
	}
	else {
		return mgi.setNamedTarget(t);
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "hand_demo_warehouse_onecamera");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	bool randomize= pnh.param<bool>("random", true);
	double wrist1_lower_limit = pnh.param<double>("wrist1_lower_limit", -0.698);
	double wrist1_upper_limit = pnh.param<double>("wrist1_upper_limit", 0.489);
	double wrist2_lower_limit = pnh.param<double>("wrist1_upper_limit", -0.489);
	double wrist2_upper_limit = pnh.param<double>("wrist2_upper_limit", 0.140);

	get_named_state= nh.serviceClient<moveit_msgs::GetRobotStateFromWarehouse>("get_robot_state", true);
	ROS_INFO("waiting for warehouse");
	get_named_state.waitForExistence();

	ROS_INFO("setting up MGI and PSI");
	moveit::planning_interface::MoveGroupInterface mgi("right_hand");
	moveit::planning_interface::PlanningSceneInterface psi;
	mgi.setPlanningTime(10);
	moveit::planning_interface::MoveGroupInterface::Plan shadow_plan;

	ros::Subscriber state_sub = nh.subscribe("/joint_states", 1, get_current_states);
	ros::Subscriber camera_sub = nh.subscribe("/webcam/image_raw", 1, One_camera_callback);

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr shadow_model = robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(shadow_model);
	robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

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
		//"open",
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
		"grasp_4_pen2",
		"grasp_5_close",
		"grasp_6_nolf",
		"grasp_no_ff",
		"grasp_parallel",
		"grasp_parallel2",
		"grasp_parallel3",
		"grasp_smallcylinder2",
		"grasp_thinobjs",
		"rock_the_world",
		"lanhua_finger",
	};

	Eigen::VectorXd w1,w2;
	w1.setLinSpaced(8,wrist1_lower_limit + 0.1,wrist1_upper_limit - 0.1);
	w2.setLinSpaced(4,wrist2_lower_limit + 0.1,wrist2_upper_limit - 0.1);

while(ros::ok()){
	for (int i = 0; i < w1.rows(); i++)
	// for (int i = 3; i < 4; i++)
	{
		for (int y = 0; y < w2.rows(); y++)
		// for (int y = 1; y < 2; y++)
		{
				ROS_INFO_STREAM("Run " << i*4+y+1 << "th wrist pose");
				// one wrist pose, for all random states
				if(randomize){
					static std::default_random_engine rnd(time(nullptr));
					shuffle (targets.begin(), targets.end(),rnd);
				}
				for( auto& t : targets ){
					ROS_INFO_STREAM("Going to state " << t);

					if(!set_named_target(mgi, t, w1(i), w2(y))){
						ROS_WARN_STREAM("Don't know state '" << t << "'. Skipping");
						continue;
					}

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
						// ROS_WARN("Collision between: %s and %s, need to allow these collisions", it->first.first.c_str(), it->first.second.c_str());
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

					if (!(planned= static_cast<bool>(mgi.plan(shadow_plan))))
					{
						ROS_WARN_STREAM("Failed to plan state '" << t<< "'");
					}

					if(!(moved= static_cast<bool>(mgi.execute(shadow_plan)))){
						ROS_WARN_STREAM("Failed to execute state '" << t<< "'");
					}
					else
					{
						ROS_INFO_STREAM(" moved to " << t);
					}

					planned = false;

					// forbid collisions again
					{
						moveit_msgs::PlanningScene scene_msg;
						scene_msg.is_diff= true;
						scene_msg.robot_state.is_diff= true;
						full_acm.getMessage(scene_msg.allowed_collision_matrix);
						psi.applyPlanningScene(scene_msg);
					}
				}
			}
		}
		ROS_ERROR("Shadow achieves all pose at this wrist pose");
		break;
	}
	return 0;
}
