/*******************************************************************************
 * Copyright (c) 2019 Iason Sarantopoulos
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/

#include <ros/ros.h>
#include <thread>
#include <autharl_core>
#include <lwr_robot/robot_sim.h>
#include <lwr_robot/robot.h>
#include <bhand_robot/robot.h>
#include <rosba_msgs/Grasp.h>
#include <std_srvs/Trigger.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>

// Moveit staff
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Global variables
std::shared_ptr<arl::robot::Robot> arm_robot;
std::shared_ptr<arl::robot::Robot> hand_robot;
std::shared_ptr<arl::viz::RVisualizer> rviz;
bool already_home = false;
std::shared_ptr<arl::controller::JointTrajectory> hand_joint_controller;
const std::string NODE_NAME = "rosba_grasper";

// Parameters
double DURATION = 3.5;
double PLANNING_TIME = 5.0;
bool REAL_ARM = false;
bool REAL_HAND = false;
std::string group_name = "lwr_ati_xtion_bhand";
Eigen::VectorXd HOME_ARM(7);
Eigen::VectorXd HOME_HAND(8);
std::string RVIZ_FRAME = "world";
std::string RVIZ_TOPIC = "/rosba_grasper_target_frame";

void config()
{
  ros::NodeHandle n("~");
  n.getParam("config/real_hw/arm", REAL_ARM);
  n.getParam("config/real_hw/hand", REAL_HAND);
  n.getParam("config/moveit/group_name", group_name);
  n.getParam("config/moveit/planning_time", PLANNING_TIME);
  n.getParam("config/moveit/duration", DURATION);

  n.getParam("config/rviz/frame", RVIZ_FRAME);
  n.getParam("config/rviz/topic", RVIZ_TOPIC);

  XmlRpc::XmlRpcValue joint;
  n.getParam("config/home/arm", joint);
  for (unsigned int i = 0; i < 7; i++)
  {
    HOME_ARM(i) = joint[i];
  }

  ROS_INFO_STREAM("Params config for " << NODE_NAME << std::endl
                   << "REAL_ARM: " << REAL_ARM << std::endl
                   << "REAL_HAND: " << REAL_HAND << std::endl
                   << "PLANNING_TIME: " << PLANNING_TIME << std::endl
                   << "DURATION: " << DURATION << std::endl
                   << "RVIZ_FRAME: " << RVIZ_FRAME << std::endl
                   << "RVIZ_TOPIC: " << RVIZ_TOPIC << std::endl
                   << "group_name: " << group_name << std::endl
                   << "arm_home_config: " << HOME_ARM.transpose() << std::endl
                   << "hand_home_config: " << HOME_HAND.transpose());
}

geometry_msgs::Pose toROS(const Eigen::Affine3d& input)
{
  geometry_msgs::Pose result;
  result.position.x = input.translation()(0);
  result.position.y = input.translation()(1);
  result.position.z = input.translation()(2);
  auto q = Eigen::Quaterniond(input.linear().matrix());
  result.orientation.x = q.x();
  result.orientation.y = q.y();
  result.orientation.z = q.z();
  result.orientation.w = q.w();
  return result;
}

Eigen::Affine3d toEigen(const geometry_msgs::Pose& input)
{
  Eigen::Affine3d result;
  result.translation() << input.position.x, input.position.y, input.position.z;
  auto quat = Eigen::Quaterniond(input.orientation.w, input.orientation.x, input.orientation.y, input.orientation.z);
  result.linear() = quat.toRotationMatrix();
  return result;
}

bool callback(rosba_msgs::Grasp::Request  &req,
              rosba_msgs::Grasp::Response &res)
{
  already_home = false;
  Eigen::Vector3d arm_init_pos = arm_robot->getTaskPosition();

  // Transform pose to world frame
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  transformStamped = tfBuffer.lookupTransform("world", req.target_pose.header.frame_id, ros::Time(0), ros::Duration(10));
  geometry_msgs::PoseStamped target_pose, target_pose1;
  tf2::doTransform(req.target_pose, target_pose, transformStamped);
  std::cout << "Pose:" << std::endl << target_pose.pose << std::endl;
  rviz->visualizeFrame(toEigen(target_pose.pose));

  // Create group_name
  moveit::planning_interface::MoveGroupInterface group(group_name);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::RobotState start_state;
  group.setPlanningTime(PLANNING_TIME);
  group.setNumPlanningAttempts(30);
  group.setGoalTolerance(0.005);
  group.setPoseTarget(target_pose);
  bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  arl::primitive::JointTrajectory trajectory(plan.trajectory_.joint_trajectory);
  trajectory.scale(DURATION / trajectory.duration());

  // Create joint trajectory controlle for executing plans
  arm_robot->setMode(arl::robot::Mode::POSITION_CONTROL);
  arl::controller::JointTrajectory trajectory_controller(arm_robot);
  trajectory_controller.reference(trajectory);
  trajectory_controller.run();

  res.success = true;
  ROS_INFO("Grasper finished.");
  return true;
}

bool goHome(std_srvs::Trigger::Request  &req,
            std_srvs::Trigger::Response &res)
{
  if (already_home)
  {
    ROS_INFO_STREAM(NODE_NAME << ": " << "goHome: Arm already home.");
    res.success = true;
    return true;
  }

  // Move arm to home position
  ROS_INFO_STREAM(NODE_NAME << ": " << "goHome: Moving arm to the home position...");
  arm_robot->setMode(arl::robot::Mode::POSITION_CONTROL);
  ros::Duration(1.0).sleep();
  arm_robot->setJointTrajectory(HOME_ARM, 8);

  already_home = true;

  res.success = true;
  ROS_INFO_STREAM(NODE_NAME << ": " << "goHome: Finished.");
  return true;
}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle n;
  config();

  // Start an async spinner for MoveIt!
  ros::AsyncSpinner spinner(2);
  spinner.start();

  rviz.reset(new arl::viz::RVisualizer(RVIZ_FRAME, RVIZ_TOPIC));

  // Create the robot after you have launch the URDF on the parameter server
  auto arm_model = std::make_shared<arl::robot::ROSModel>("/robot_description", "autharl_lwr_model");
  auto hand_model = std::make_shared<arl::robot::ROSModel>("/robot_description", "autharl_bhand_model");

  // auto arm_model = std::make_shared<lwr::robot::Model>("/robot_description");
  // auto hand_model  = std::make_shared<arl::bhand::Model>("/robot_description");

  // Create a simulated robot
  if (REAL_ARM)
  {
    arm_robot.reset(new arl::lwr::Robot(arm_model));
  }
  else
  {
    arm_robot.reset(new arl::lwr::RobotSim(arm_model, 1e-3));
  }

  if (REAL_HAND)
  {
    hand_robot.reset(new arl::bhand::Robot(hand_model));
  }
  else
  {
    hand_robot.reset(new arl::robot::RobotSim(hand_model, 1e-3));
  }

  std::shared_ptr<arl::robot::Sensor> sensor;

  // Create a visualizater for see the result in rviz
  auto arm_rviz = std::make_shared<arl::viz::RosStatePublisher>(arm_robot, "/autharl_joint_state", 50, "world", "rosba_arm_viz");
  auto hand_rviz = std::make_shared<arl::viz::RosStatePublisher>(hand_robot, "/autharl_joint_state_tool", 50, "world", "rosba_hand_viz");

  std::thread arm_viz_thread(&arl::viz::RosStatePublisher::run, arm_rviz);
  std::thread hand_viz_thread(&arl::viz::RosStatePublisher::run, hand_rviz);

  ROS_INFO_STREAM(NODE_NAME << ": " << "main: Moving arm/hand to home position.");
  hand_joint_controller.reset(new arl::controller::JointTrajectory(hand_robot, arl::robot::Controller::Timing::DYNAMIC));
  hand_robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
  hand_joint_controller->reference({3.14, 0, 0, 0, 0, 0, 0, 0}, 7);
  std::thread hand_joint_controller_thread(&arl::controller::JointTrajectory::run, hand_joint_controller);
  arm_robot->setMode(arl::robot::Mode::POSITION_CONTROL);
  arm_robot->setJointTrajectory(HOME_ARM, 8);
  hand_joint_controller_thread.join();
  hand_robot->setMode(arl::robot::Mode::STOPPED);

  ros::ServiceServer service = n.advertiseService("grasp", callback);
  ros::ServiceServer go_home_srv = n.advertiseService("go_home", goHome);
  ROS_INFO_STREAM(NODE_NAME << ": " << "main: Ready to grasp objects.");

  arm_viz_thread.join();
  hand_viz_thread.join();
  return 0;
}
