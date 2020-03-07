#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "utils.hpp"

int main(int argc, char** argv){
	ros::init(argc, argv, "mtc_grasp_bottle");

	moveit::planning_interface::MoveGroupInterface mgi("right_fingers");
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit::planning_interface::MoveGroupInterface mgi_arm("right_arm_and_hand");

	// open hand
	mgi.setJointValueTarget({ });
	mgi.move();

	mgi_arm.setJointValueTarget({});
	mgi_arm.move();

	ros::Duration(3.0).sleep();

	moveit_msgs::AttachedCollisionObject aco;
	collisionObjectFromResource(aco.object, "still_water", "package://mtc_pour/meshes/bottle.stl");
	aco.link= "rh_palm";
	aco.object.header.frame_id= "rh_palm";
	auto& bottle_pose= aco.object.mesh_poses.back();
	bottle_pose.orientation.w= 1;
	bottle_pose.position.x= 0.0;
	bottle_pose.position.y= -0.05;
	bottle_pose.position.z= 0.07;

	aco.object.subframe_names.emplace_back("spout");
	aco.object.subframe_poses.emplace_back();
	aco.object.subframe_poses.back().orientation.w= 1.0;
	aco.object.subframe_poses.back().position.z= computeMeshHeight(aco.object.meshes[0])/2;

	psi.applyAttachedCollisionObject(aco);

	// close hand
	mgi.setJointValueTarget({ });
	mgi.move();

	return 0;
}
