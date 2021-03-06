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

#include <actionlib/client/simple_action_client.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>


#include "utils.hpp"

#include <memory>

using namespace moveit::task_constructor;

int main(int argc, char** argv){
	ros::init(argc, argv, "mtc_pouring");

	ros::AsyncSpinner spinner(2);
	spinner.start();

	{
		const double table_height= .7;

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
		setupObjects(bottle, glass);
	}

	Task t;
	t.loadRobotModel();

	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	auto constraint_sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	constraint_sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	//// don't spill liquid
	//moveit_msgs::Constraints upright_constraint;
	//upright_constraint.name = "l_gripper_tool_frame:upright";
	//upright_constraint.orientation_constraints.resize(1);
	//{
	//	moveit_msgs::OrientationConstraint& c= upright_constraint.orientation_constraints[0];
	//	c.link_name= "l_gripper_tool_frame";
	//	c.header.frame_id= "base_footprint";
	//	c.orientation.w= 1;
	//	c.absolute_x_axis_tolerance= 0.6;
	//	c.absolute_y_axis_tolerance= 0.6;
	//	c.absolute_z_axis_tolerance= M_PI;
	//	//c.absolute_x_axis_tolerance= 0.65;
	//	//c.absolute_y_axis_tolerance= 0.65;
	//	//c.absolute_z_axis_tolerance= M_PI;
	//	c.weight= 1.0;
	//}

	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScaling(.3);
	cartesian_planner->setMaxAccelerationScaling(.3);
	cartesian_planner->setStepSize(.006);

	t.setProperty("group", "right_arm_and_hand");
	t.setProperty("eef", "right_hand");
	t.setProperty("gripper", "right_fingers");

	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		t.add(std::move(_current_state));
	}

	Stage* current_state= nullptr;
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper->object collision");
		stage->allowCollisions("still_water", t.getRobotModel()->getJointModelGroup("right_hand")->getLinkModelNamesWithCollisionGeometry(), true);
		current_state= stage.get();
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::Connect>("move to pre-pour pose", stages::Connect::GroupPlannerVector{{"right_arm_and_hand", constraint_sampling_planner}});
		stage->setTimeout(5.0);
		//stage->setPathConstraints(upright_constraint);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	{
		// TODO: add attached objects as markers to debug marker
		auto stage = std::make_unique<stages::GeneratePose>("pose above glass");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "glass";
		p.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,0/*-M_PI/4*/);
		p.pose.position.z= .24;
		stage->setPose(p);
		stage->properties().configureInitFrom(Stage::PARENT);

		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("pre-pour pose", std::move(stage) );
		wrapper->setMaxIKSolutions(8);
		wrapper->setIKFrame("still_water");
		wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"});
		wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
		t.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<mtc_pour::PourInto>("pour");
		stage->setBottle("bottle");
		stage->setContainer("glass");
		stage->setPourOffset(Eigen::Vector3d(0,0.018,0.050));
		{
			geometry_msgs::Vector3Stamped pouring_axis;
			pouring_axis.header.frame_id= "base_footprint";
			pouring_axis.vector.x=1.0;
			stage->setPouringAxis(pouring_axis);
		}
		stage->setTiltAngle(2.0);
		stage->setPourDuration(ros::Duration(2.0));
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("move back", sampling_planner);
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setGoal({ });
		t.add(std::move(stage));
	}

	t.enableIntrospection();

	ROS_INFO_STREAM( t );

	try {
		t.plan(2);

		std::cout << "waiting for <enter>" << std::endl;
		std::cin.get();
	}
	catch(InitStageException& e){
		ROS_ERROR_STREAM(e);
	}

	if(t.numSolutions() > 0){
		ROS_INFO("executing first solution");

		actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> ac("execute_task_solution", true);
		ac.waitForServer();

		moveit_task_constructor_msgs::ExecuteTaskSolutionGoal goal;
		t.solutions().front()->fillMessage(goal.solution);
		ROS_INFO_STREAM("solution has " << goal.solution.sub_trajectory.size() << " subtrajectories. Trimmed to 19.");
		goal.solution.sub_trajectory.resize(19);
		ac.sendGoal(goal);
		ac.waitForResult();

		ROS_INFO_STREAM("action returned: " << ac.getState().toString());
	}

	return 0;
}
