#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Float64MultiArray.h>
//#include <my_package/coordinate.h>


double x, y, z;
double roll, pitch, yaw;

class Localization{

public:
   double pos;
   void servo(const sensor_msgs::JointState::ConstPtr& arm_step); 


};


void Localization::servo(const sensor_msgs::JointState::ConstPtr& arm_step){

   pos = arm_step->position[0];
}

void coordinatesCallback(const geometry_msgs::Point& msg){
    x = msg.x;
    y = msg.y;
    z = msg.z;
    
}
using namespace std;

int main(int argc, char** argv){


  float x1[]={-0.1601857,-0.2601857,0.06017,0.1601857,-0.1601857,-0.1601857,-0.2601857,0.06017,0.1601857,-0.1601857};
  float y1[]={0.443768,0.3601857,0.26017,0.3601857,0.443768,0.443768,0.3601857,0.26017,0.3601857,0.443768};
  float z1[]={-0.210889,-0.210889,-0.310889,-0.350889,-0.210889,-0.210889,-0.210889,-0.310889,-0.350889,-0.210889};

  //int i = 0;
  int count = 0;
  
  ros::init(argc, argv, "pose_goal");
  ros::NodeHandle node_handle;
  Localization localization;
  ros::Subscriber sub = node_handle.subscribe<sensor_msgs::JointState> ("joint_states",1, &Localization::servo,&localization);
  //ros::Subscriber coordinate_sub = node_handle.subscribe("tea_coords", 1000, coordinatesCallback);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Rate rate(0.5);
  //ros::Duration(5).sleep();
   
  moveit::planning_interface::MoveGroupInterface move_group_interface("arm");
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper("hand");
  const robot_state::JointModelGroup *joint_model_group =
    move_group_interface.getCurrentState()->getJointModelGroup("arm");
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;
  
  //pluck position
  move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("pluck"));
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface.move();
    ros::Duration(2).sleep();
    
    //gripper open
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface_gripper.move();
    ros::Duration(0.5).sleep();
	  
       
  
 while(true){
   //ros::Subscriber coordinate_sub = node_handle.subscribe("tea_coords", 1000, coordinatesCallback);
   ros::Duration(2).sleep();
	  
   //if(x != 0 && y != 0 && z != 0){
      for (int i=0;i<5;i++){
 
      
      //gripper open
      move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("close"));
      success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      move_group_interface_gripper.move();
      ros::Duration(0.5).sleep();
      
            
      
      //pluck position
  move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("pluck"));
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group_interface.move();
    ros::Duration(2).sleep();
      
     
	  
      //move to given coordinate
      //move_group_interface.setEndEffectorLink("Link_04");
      move_group_interface.setPlanningTime(20.0);
      move_group_interface.setGoalJointTolerance(1e-4);
      move_group_interface.setGoalPositionTolerance(1e-4);
      move_group_interface.setGoalOrientationTolerance(1e-3);
	  
      //cout << "Enter Value x";
      //cin >> x_input;
      //cout << "Enter Value y";
      //cin >> y_input;
      //cout << "Enter Value z";
      //cin >> z_input;
	  
	  
      //ROS_INFO_STREAM("x: " << x<< " y: " << z<< " z: " << y );
      geometry_msgs::PoseStamped current_pose;
      current_pose = move_group_interface.getCurrentPose();
      geometry_msgs::Pose target_pose;
      target_pose.position.x = -0.1601857;
      target_pose.position.y = 0.2;
      target_pose.position.z = -0.4;
	  
      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY( 1.570800,0,0); 
      ROS_INFO_STREAM("x: " << myQuaternion.getX() << " y: " << myQuaternion.getY() << " z: " << myQuaternion.getZ() << " w: " << myQuaternion.getW());
      myQuaternion= myQuaternion.normalize();
      target_pose.orientation.w = myQuaternion.getW()  ;
      target_pose.orientation.x = myQuaternion.getX()  ;
      target_pose.orientation.y = myQuaternion.getY() ;
      target_pose.orientation.z = myQuaternion.getZ() ;
	  
      move_group_interface.setPoseTarget(target_pose);
      success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	  
      if (success)
      {
	  ROS_INFO("[point_move/pose_goal] Planning OK. Proceeding.");
      }
      else
      {
	  ROS_WARN("[point_move/pose_goal] Planning failed. Shutting down.");
	  ros::shutdown();
	  return 0;
      }
	  

      move_group_interface.move();
      ros::Duration(1).sleep();
      
      //TESTINGGGGGGGGGGGGGGGGGGGGG
      /*ROS_INFO("pos:%.2f",localization.pos);
      tf2::Quaternion myQuaternion1;
      myQuaternion1.setRPY( 1.570800,0,localization.pos/57.2958); 
      ROS_INFO_STREAM("x: " << myQuaternion1.getX() << " y: " << myQuaternion1.getY() << " z: " << myQuaternion1.getZ() << " w: " << myQuaternion1.getW());
      myQuaternion= myQuaternion1.normalize();
      target_pose.orientation.w = myQuaternion1.getW()  ;
      target_pose.orientation.x = myQuaternion1.getX()  ;
      target_pose.orientation.y = myQuaternion1.getY() ;
      target_pose.orientation.z = myQuaternion1.getZ() ;
	  
      move_group_interface.setPoseTarget(target_pose);
      success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	  
      if (success)
      {
	  ROS_INFO("[point_move/pose_goal] Planning OK. Proceeding.");
      }
      else
      {
	  ROS_WARN("[point_move/pose_goal] Planning failed. Shutting down.");
	  ros::shutdown();
	  return 0;
      }
	  

      move_group_interface.execute(my_plan);
      ros::Duration(5).sleep();*/
	  
      //gripper close
      move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
      success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      move_group_interface_gripper.move();
      ros::Duration(0.5).sleep();
      
	  
      //drop position
      if (count%3==0){
      move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("drop"));
      success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success)
      {
	 ROS_INFO("[point_move/pose_goal] Planning OK. Proceeding.");
      }
      else
      {
	 ROS_WARN("[point_move/pose_goal] Planning failed. Shutting down.");
	 ros::shutdown();
	 return 0;
      }
      move_group_interface.move();
      ros::Duration(0.5).sleep();
      }else if(count%3==1){
      move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("drop1"));
      success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success)
      {
	 ROS_INFO("[point_move/pose_goal] Planning OK. Proceeding.");
      }
      else
      {
	 ROS_WARN("[point_move/pose_goal] Planning failed. Shutting down.");
	 ros::shutdown();
	 return 0;
      }
      move_group_interface.move();
      ros::Duration(0.5).sleep();
      }else{
      move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("drop2"));
      success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success)
      {
	 ROS_INFO("[point_move/pose_goal] Planning OK. Proceeding.");
      }
      else
      {
	 ROS_WARN("[point_move/pose_goal] Planning failed. Shutting down.");
	 ros::shutdown();
	 return 0;
      }
      move_group_interface.move();
      ros::Duration(0.5).sleep();
      }
  
      count++;
	  
      } 

      /* need to edit following codes when camera is connected, right now this confuses the programe sending the same command again and again*/
      
      
      //count=0;
	    
    //}
	  
    /*else{
	  
       if(count == 0){
	  //rest position
	  move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("rest"));
	  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	  if (success)
	  {
	    ROS_INFO("[point_move/pose_goal] Planning OK. Proceeding.");
	  }
	  else
	  {
	    ROS_WARN("[point_move/pose_goal] Planning failed. Shutting down.");
	    ros::shutdown();
	    return 0;
	  }
	  move_group_interface.move();
	  //ros::Duration(2).sleep();
       }
	  
       else{
	   ROS_INFO("Already in rest position");
       }
       
    }*/
  } 
	  	  
	  //ros::shutdown();
	  //return 0;
}

