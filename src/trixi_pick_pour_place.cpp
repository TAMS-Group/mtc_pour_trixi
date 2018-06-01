#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h>

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <moveit/task_constructor/stages/pour_into.h>

#include <moveit/task_constructor/stages/modify_planning_scene.h>

#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include "utils.hpp"

#include <memory>

using namespace moveit::task_constructor;

int main(int argc, char** argv){
	ros::init(argc, argv, "mtc_pouring");

	ros::AsyncSpinner spinner(2);
	spinner.start();

	{
		const double table_height= .55;

		geometry_msgs::PoseStamped bottle;
		bottle.header.frame_id= "base_link";
		bottle.pose.position.x= 0.6;
		bottle.pose.position.y= 0.13;
		bottle.pose.position.z= table_height;
		bottle.pose.orientation.w= 1.0;

		geometry_msgs::PoseStamped glass;
		glass.header.frame_id= "base_link";
		glass.pose.position.x= 0.65;
		glass.pose.position.y= 0.00;
		glass.pose.position.z= table_height;
		glass.pose.orientation.w= 1.0;

		// center of table surface can be the glass pose
		geometry_msgs::PoseStamped tabletop= glass;

		setupTable(tabletop);
		setupObjects(bottle, glass, "package://mtc_pour/meshes/small_bottle.stl");
	}

	Task t;
	t.loadRobotModel();

	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	auto constraint_sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	constraint_sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	// don't spill liquid
	moveit_msgs::Constraints upright_constraint;
	upright_constraint.name = "l_gripper_tool_frame:upright";
	upright_constraint.orientation_constraints.resize(1);
	{
		moveit_msgs::OrientationConstraint& c= upright_constraint.orientation_constraints[0];
		c.link_name= "l_gripper_tool_frame";
		c.header.frame_id= "base_footprint";
		c.orientation.w= 1;
		c.absolute_x_axis_tolerance= 0.6;
		c.absolute_y_axis_tolerance= 0.6;
		c.absolute_z_axis_tolerance= M_PI;
		//c.absolute_x_axis_tolerance= 0.65;
		//c.absolute_y_axis_tolerance= 0.65;
		//c.absolute_z_axis_tolerance= M_PI;
		c.weight= 1.0;
	}

	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScaling(.3);
	cartesian_planner->setMaxAccelerationScaling(.3);
	cartesian_planner->setStepSize(.006);

	t.setProperty("group", "left_arm");
	t.setProperty("eef", "left_gripper");
	t.setProperty("gripper", "left_gripper");

	Stage* current_state= nullptr;
	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		current_state= _current_state.get();
		t.add(std::move(_current_state));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("open gripper", sampling_planner);
		stage->properties().property("group").configureInitFrom(Stage::PARENT, {"gripper"});
		stage->setGoal("open");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::Connect>("move to pre-grasp pose", stages::Connect::GroupPlannerVector {{"left_arm", sampling_planner}});
		stage->setTimeout(10.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
		stage->properties().set("marker_ns", "approach");
		stage->properties().set("link", "l_gripper_tool_frame");
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.07, .20);

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "l_gripper_tool_frame";
		vec.vector.x = 1.0;
		stage->along(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GenerateGraspPose>("grasp work space pose");
		stage->properties().configureInitFrom(Stage::PARENT);
		stage->setNamedPose("open");
		stage->setObject("bottle");
		stage->setAngleDelta(M_PI/16);

		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose", std::move(stage) );
		wrapper->setMaxIKSolutions(1);
		wrapper->setTimeout(0.05);
		wrapper->setIgnoreCollisions(true);
		wrapper->setIKFrame("l_gripper_tool_frame");
		wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
		t.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper->object collision");
		stage->allowCollisions("bottle", t.getRobotModel()->getJointModelGroup("left_gripper")->getLinkModelNamesWithCollisionGeometry(), true);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("close gripper", sampling_planner);
		stage->properties().property("group").configureInitFrom(Stage::PARENT, "gripper");
		stage->setGoal("closed");
		t.add(std::move(stage));
	}

	Stage* object_grasped= nullptr;
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
		stage->attachObject("bottle", "l_gripper_tool_frame");
		object_grasped= stage.get();
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.08,.13);
		stage->setIKFrame("l_gripper_tool_frame");

		stage->properties().set("marker_ns", "lift");

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id= "base_footprint";
		vec.vector.z= 1.0;
		stage->along(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::Connect>("move to pre-pour pose", stages::Connect::GroupPlannerVector{{"left_arm", constraint_sampling_planner}});
		stage->setTimeout(5.0);
		stage->setPathConstraints(upright_constraint);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("pose above glass");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "glass";
		p.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,0/*-M_PI/4*/);
		p.pose.position.z= .19;
		stage->setPose(p);
		stage->properties().configureInitFrom(Stage::PARENT);

		stage->setMonitoredStage(object_grasped);

		auto wrapper = std::make_unique<stages::ComputeIK>("pre-pour pose", std::move(stage) );
		wrapper->setMaxIKSolutions(8);
		wrapper->setIKFrame("l_gripper_tool_frame");
		wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
		t.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<mtc_pour::PourInto>("pouring");
		stage->setBottle("bottle");
		stage->setContainer("glass");
		stage->setPourOffset(Eigen::Vector3d(0,0.015,0.035));
		stage->setTiltAngle(2.0);
		stage->setPourDuration(ros::Duration(2.0));
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	//// PLACE

	{
		auto stage = std::make_unique<stages::Connect>("move to pre-place pose", stages::Connect::GroupPlannerVector{{"left_arm", sampling_planner}});
		stage->setTimeout(5.0);
//		stage->setPathConstraints(upright_constraint);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("put down object", cartesian_planner);
		stage->properties().set("marker_ns", "approach_place");
		stage->properties().set("link", "l_gripper_tool_frame");
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.08, .13);

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "l_gripper_tool_frame";
		vec.vector.z = -1.0;
		stage->along(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("place pose");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "table";
		p.pose.position.x=  0.1;
		p.pose.position.y=  0.3;
		p.pose.position.z=  0.20; // 0.14
		p.pose.orientation.w= 1;
		stage->setPose(p);
		stage->properties().configureInitFrom(Stage::PARENT);

		stage->setMonitoredStage(object_grasped);

		auto wrapper = std::make_unique<stages::ComputeIK>("place pose kinematics", std::move(stage) );
		wrapper->setMaxIKSolutions(8);
		wrapper->setIKFrame("l_gripper_tool_frame");
		wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
		t.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("release object", sampling_planner);
		stage->properties().property("group").configureInitFrom(Stage::PARENT, "gripper");
		stage->setGoal("open");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper->object collision");
		stage->allowCollisions("bottle", t.getRobotModel()->getJointModelGroup("left_gripper")->getLinkModelNamesWithCollisionGeometry(), false);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
		stage->detachObject("bottle", "l_gripper_tool_frame");
		object_grasped= stage.get();
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.10,.20);
		stage->setIKFrame("l_gripper_tool_frame");

		stage->properties().set("marker_ns", "post-place");

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id= "l_gripper_tool_frame";
		vec.vector.x=  -1.0;
		vec.vector.z= 0.75;
		stage->along(vec);
		t.add(std::move(stage));
	}

	//{
	//	auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
	//	stage->properties().configureInitFrom(Stage::PARENT, {"group"});
	//	stage->setGoal("");
	//	t.add(std::move(stage));
	//}

	t.enableIntrospection();

	//ros::NodeHandle nh("~");
	//bool execute= nh.param<bool>("execute", false);

	ROS_INFO_STREAM( t );

	try {
		t.plan(10);

		std::cout << "waiting for <enter>" << std::endl;
		std::cin.get();
	}
	catch(InitStageException& e){
		ROS_ERROR_STREAM(e);
	}

	return 0;
}
