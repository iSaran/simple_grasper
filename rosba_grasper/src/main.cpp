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
std::shared_ptr<arl::robot::Robot> arm_robot, hand_robot, arm_viz_robot, hand_viz_robot;
std::shared_ptr<arl::viz::RVisualizer> rviz;
std::shared_ptr<arl::viz::RosStatePublisher> arm_rviz, hand_rviz;
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
Eigen::VectorXd HOME_HAND(8), PREGRASP_HAND(8);
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
  n.getParam("config/home/hand", joint);
  for (unsigned int i = 0; i < 8; i++)
  {
    HOME_HAND(i) = joint[i];
  }

  n.getParam("config/home/pregrasp", joint);
  for (unsigned int i = 0; i < 8; i++)
  {
    PREGRASP_HAND(i) = joint[i];
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
                   << "hand_home_config: " << HOME_HAND.transpose() << std::endl
                   << "hand_pregrasp_config: " << PREGRASP_HAND.transpose());
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

bool setJointTrajectoryParallel(const Eigen::VectorXd& arm, const Eigen::VectorXd& hand)
{
  hand_robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
  hand_joint_controller->reference(hand.toArma(), 7);
  std::thread hand_joint_controller_thread(&arl::controller::JointTrajectory::run, hand_joint_controller);
  arm_robot->setMode(arl::robot::Mode::POSITION_CONTROL);
  arm_robot->setJointTrajectory(arm, 10);
  hand_joint_controller_thread.join();
  hand_robot->setMode(arl::robot::Mode::STOPPED);
}

bool setJointTrajectory(const arl::primitive::JointTrajectory& arm, const Eigen::VectorXd& hand, bool viz_first=false)
{
  if (viz_first)
  {
    // Create joint trajectory controller for executing plans
    arm_rviz->setRobotPointer(arm_viz_robot);
    hand_rviz->setRobotPointer(hand_viz_robot);

    ros::Duration(1.0).sleep();

    // arm_viz_robot->setJointPosition(arm_robot->getJointPosition());
    // hand_viz_robot->setJointPosition(hand_robot->getJointPosition());

    arm_viz_robot->setMode(arl::robot::Mode::POSITION_CONTROL);
    arl::controller::JointTrajectory trajectory_controller(arm_viz_robot);
    trajectory_controller.reference(arm);
    trajectory_controller.run();

    hand_viz_robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
    arl::controller::JointTrajectory hand_joint_controller_(hand_viz_robot);
    hand_joint_controller_.reference(hand.toArma(), 7);
    hand_joint_controller_.run();
    hand_viz_robot->setMode(arl::robot::Mode::STOPPED);

    int x;
    std::cout << "Execute? (0: No, 1: Yes): ";
    std::cin >> x; // Get user input from the keyboard

    arm_rviz->setRobotPointer(arm_robot);
    hand_rviz->setRobotPointer(hand_robot);

    if (x == 0)
    {
      return true;
    }
  }

  arm_robot->setMode(arl::robot::Mode::POSITION_CONTROL);
  arl::controller::JointTrajectory trajectory_controller(arm_robot);
  trajectory_controller.reference(arm);
  trajectory_controller.run();

  hand_robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
  arl::controller::JointTrajectory hand_joint_controller_(hand_robot);
  hand_joint_controller_.reference(hand.toArma(), 7);
  hand_joint_controller_.run();
  hand_robot->setMode(arl::robot::Mode::STOPPED);
}

bool setJointTrajectories(const std::vector<arl::primitive::JointTrajectory*>& arm, const std::vector<Eigen::VectorXd>& hand, bool viz_first=false)
{
  if (viz_first)
  {
    // Create joint trajectory controller for executing plans
    arm_rviz->setRobotPointer(arm_viz_robot);
    hand_rviz->setRobotPointer(hand_viz_robot);

    ros::Duration(1.0).sleep();

    for (unsigned int i = 0; i < arm.size(); i++)
    {
      arm_viz_robot->setMode(arl::robot::Mode::POSITION_CONTROL);
      arl::controller::JointTrajectory trajectory_controller(arm_viz_robot);
      trajectory_controller.reference(*arm.at(i));
      trajectory_controller.run();

      hand_viz_robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
      arl::controller::JointTrajectory hand_joint_controller_(hand_viz_robot);
      hand_joint_controller_.reference(hand.at(i).toArma(), 7);
      hand_joint_controller_.run();
      hand_viz_robot->setMode(arl::robot::Mode::STOPPED);
    }

    // arm_viz_robot->setJointPosition(arm_robot->getJointPosition());
    // hand_viz_robot->setJointPosition(hand_robot->getJointPosition());

    int x;
    std::cout << "Execute? (0: No, 1: Yes): ";
    std::cin >> x; // Get user input from the keyboard

    arm_rviz->setRobotPointer(arm_robot);
    hand_rviz->setRobotPointer(hand_robot);

    if (x == 0)
    {
      return true;
    }
  }

  for (unsigned int i = 0; i < arm.size(); i++)
  {
    arm_robot->setMode(arl::robot::Mode::POSITION_CONTROL);
    arl::controller::JointTrajectory trajectory_controller(arm_robot);
    trajectory_controller.reference(*arm.at(i));
    trajectory_controller.run();

    hand_robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
    arl::controller::JointTrajectory hand_joint_controller_(hand_robot);
    hand_joint_controller_.reference(hand.at(i).toArma(), 7);
    hand_joint_controller_.run();
    hand_robot->setMode(arl::robot::Mode::STOPPED);
  }
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


  // Got to pregrasp config
  hand_robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
  arl::controller::JointTrajectory hand_joint_controller_(hand_robot);
  hand_joint_controller_.reference(PREGRASP_HAND.toArma(), 7);
  hand_joint_controller_.run();
  hand_robot->setMode(arl::robot::Mode::STOPPED);
  ros::Duration(1.0).sleep();

  // Create group_name
  moveit::planning_interface::MoveGroupInterface group(group_name);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::RobotState start_state;
  group.setPlanningTime(PLANNING_TIME);
  group.setNumPlanningAttempts(1000);
  group.setGoalTolerance(0.01);
  std::vector<arl::primitive::JointTrajectory*> trajectory(req.grasp.size());
  std::vector<Eigen::VectorXd> hand_configs(req.grasp.size());

  // Transform pose to world frame
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped target_pose;
  for (unsigned int i = 0; i < req.grasp.size(); i++)
  {
    transformStamped = tfBuffer.lookupTransform("world", req.grasp[i].target_pose.header.frame_id, ros::Time(0), ros::Duration(10));
    tf2::doTransform(req.grasp[i].target_pose, target_pose, transformStamped);
    // std::cout << "Pose:" << std::endl << target_pose.pose << std::endl;
    rviz->visualizeFrame(toEigen(target_pose.pose));

    if (i > 0)
    {
      start_state.joint_state.name =  plan.trajectory_.joint_trajectory.joint_names;
      start_state.joint_state.position = plan.trajectory_.joint_trajectory.points[plan.trajectory_.joint_trajectory.points.size() - 1].positions;
      group.setStartState(start_state);
    }

    group.setPoseTarget(target_pose);
    bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (!success)
    {
      ROS_ERROR_STREAM("Failed to plan pose n." << i);
      res.success = false;
      return false;
    }

    // arl::primitive::JointTrajectory trajectory(plan.trajectory_.joint_trajectory);
    trajectory.at(i) = new arl::primitive::JointTrajectory(plan.trajectory_.joint_trajectory);
    trajectory.at(i)->scale(DURATION / trajectory.at(i)->duration());

    Eigen::VectorXd joint(8);
    for (unsigned int j = 0; j < 8; j++)
    {
        joint(i) = req.grasp[i].hand_joint_config[i];
    }
    hand_configs.push_back(joint);
  }

  setJointTrajectories(trajectory, hand_configs, true);

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
  ROS_INFO_STREAM(NODE_NAME << ": " << "main: Moving arm/hand to home position.");
  setJointTrajectoryParallel(HOME_ARM, HOME_HAND);
  ros::Duration(1.0).sleep();
  already_home = true;
  res.success = true;
  ROS_INFO_STREAM(NODE_NAME << ": " << "goHome: Finished.");
  return true;
}

bool goPregrasp(std_srvs::Trigger::Request  &req,
                std_srvs::Trigger::Response &res)
{
  ROS_INFO_STREAM(NODE_NAME << ": " << "main: Moving hand to pregrasp position.");
  already_home = false;
  hand_robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
  arl::controller::JointTrajectory hand_joint_controller_(hand_robot);
  hand_joint_controller_.reference(PREGRASP_HAND.toArma(), 7);
  hand_joint_controller_.run();
  hand_robot->setMode(arl::robot::Mode::STOPPED);
  ros::Duration(1.0).sleep();
  ROS_INFO_STREAM(NODE_NAME << ": " << "goPregrasp: Finished.");
  res.success = true;
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
  if (REAL_HAND)
  {
    hand_robot.reset(new arl::bhand::Robot(hand_model));
  }
  else
  {
    hand_robot.reset(new arl::robot::RobotSim(hand_model, 1e-3, false));
  }

  if (REAL_ARM)
  {
    arm_robot.reset(new arl::lwr::Robot(arm_model));
  }
  else
  {
    arm_robot.reset(new arl::lwr::RobotSim(arm_model, 1e-3));
  }
  arm_viz_robot.reset(new arl::robot::RobotSim(arm_model, 1e-3));
  hand_viz_robot.reset(new arl::robot::RobotSim(hand_model, 1e-3, false));

  std::shared_ptr<arl::robot::Sensor> sensor;
  // Dynamic timing has problems with visualization.
  //hand_joint_controller.reset(new arl::controller::JointTrajectory(hand_robot, arl::robot::Controller::Timing::DYNAMIC));
  hand_joint_controller.reset(new arl::controller::JointTrajectory(hand_robot));

  // Create a visualizer for see the result in rviz
  arm_rviz.reset(new arl::viz::RosStatePublisher(arm_robot, "/autharl_joint_state", 50, "world"));
  hand_rviz.reset(new arl::viz::RosStatePublisher(hand_robot, "/autharl_joint_state_tool", 50, "world"));

  std::thread arm_viz_thread(&arl::viz::RosStatePublisher::run, arm_rviz);
  std::thread hand_viz_thread(&arl::viz::RosStatePublisher::run, hand_rviz);

  setJointTrajectoryParallel(HOME_ARM, HOME_HAND);

  ros::ServiceServer service = n.advertiseService("grasp", callback);
  ros::ServiceServer go_home_srv = n.advertiseService("go_home", goHome);
  ros::ServiceServer go_pregrasp_srv = n.advertiseService("go_pregrasp", goPregrasp);
  ROS_INFO_STREAM(NODE_NAME << ": " << "main: Ready to grasp objects.");

  arm_viz_thread.join();
  hand_viz_thread.join();
  return 0;
}
