/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/transform_broadcaster.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  std::string robot_name;
  std::string base_link;
  double x, z, y, qua_x,qua_y,qua_z,qua_w;
  node_handle.param("robot_name", robot_name, std::string("i5aubo1"));
  // node_handle.param("robot_name", robot_name, std::string("wormRobot"));
  node_handle.param("pedestal_Link", base_link, std::string("pedestal_Link"));
  node_handle.param("x", x, -0.6);
  node_handle.param("y", y, 0.5);
  node_handle.param("z", z, 0.5);
  node_handle.param("qua_x", qua_x, 0.0);
  node_handle.param("qua_y", qua_y, -0.6);
  node_handle.param("qua_z", qua_z, 0.0);
  node_handle.param("qua_w", qua_w, 0.8);
  // ros::Rate r(1);

  
  // .. _RobotModelLoader:
  //     http://docs.ros.org/kinetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());


  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("i5aubo1");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();


  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the Panda arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  // for (std::size_t i = 0; i < joint_names.size(); ++i)
  // {
  //   ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  // }

  // Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  /* Set one joint in the Panda arm outside its joint limit */
  // joint_values[0] = 5.57;
  // kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  /* Check whether any joint is outside its joint limits */
  // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  
  // Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // We can now solve inverse kinematics (IK) for the Panda robot.
  // To solve IK, we will need the following:
  //
  //  * The desired pose of the end-effector (by default, this is the last link in the "i5aubo1" chain):
  //    end_effector_state that we computed in the step above.
  //  * The number of attempts to be made at solving IK: 10
  //  * The timeout for each attempt: 0.1 s
  
  geometry_msgs::Pose end_effector_state;
  end_effector_state.position.x = x;
  end_effector_state.position.y = y;
  end_effector_state.position.z = z;
  end_effector_state.orientation.x = qua_x;
  end_effector_state.orientation.y = qua_y;
  end_effector_state.orientation.z = qua_z;
  end_effector_state.orientation.w = qua_w;

  std::size_t attempts = 10;
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, attempts, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  // Get the Jacobian
  // ^^^^^^^^^^^^^^^^
  // We can also get the Jacobian from the :moveit_core:`RobotState`.
  // Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  // Eigen::MatrixXd jacobian;
  // kinematic_state->getJacobian(joint_model_group,
  //                              kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
  //                              reference_point_position, jacobian);
  // ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
