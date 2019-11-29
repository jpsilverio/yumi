#ifndef __YUMI_TEST_CONTROLLERS_H
#define __YUMI_TEST_CONTROLLERS_H

#include <cstdlib>

#include <chrono>
#include <iostream>
#include <signal.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <armadillo>
#include <pose_MPC.h>

using namespace std;

int num_joints = 14;
int num_joints_arm = 7;
int test_joint_number = 7;      // this is the joint that is being controlled in the original example (1- first shoulder joint, 2- second shoulder joint, etc, no gripper)
int left_joint_state_idx = 2*test_joint_number;
int right_joint_state_idx = 2*test_joint_number + 1;

// Messages for ROS topics
sensor_msgs::JointState joints_state;   // msg containing the states of all joints
geometry_msgs::Pose left_hand_desired_pose;
geometry_msgs::Pose right_hand_desired_pose;
std::vector<std_msgs::Float64> left_command(num_joints_arm);    // vector of msgs containing individual control commands (current implementation doesn't allow for simpler)
std::vector<std_msgs::Float64> right_command(num_joints_arm);
std::vector<std_msgs::Float64> left_joint_pos(num_joints_arm);
std::vector<std_msgs::Float64> right_joint_pos(num_joints_arm);

// Publishers
std::vector<ros::Publisher> left_controller_pub(num_joints_arm);                // I want to control all the joints now so I need to publish in all topics... Is there a simpler way?
std::vector<ros::Publisher> right_controller_pub(num_joints_arm);

// Subscribers
ros::Subscriber sub;
ros::Subscriber sub_control_mode; // Subscribe to topic that says to control either using KDL solver or manual pinv
ros::Subscriber sub_left_pose;
ros::Subscriber sub_right_pose;

KDL::Vector desired_pos_left(0.2,0.5,0.6);
KDL::Vector desired_pos_right(0.2,-0.5,0.6);
KDL::Rotation desired_rot_left;		// initialized with identity
KDL::Rotation desired_rot_right;
arma::vec xTar(7,arma::fill::zeros),x(7,arma::fill::zeros);


std::string control_mode = "";
bool restart_plan = true;
bool g_quit;

void publish_commands();
void print_joint_values(const KDL::JntArray& q);
void print_joint_values(const arma::vec& q);
void print_Jacobian(KDL::Jacobian J);
void quitRequested(int sig);
void rearrange_YuMi_joints(std::vector<std_msgs::Float64>& joint_pos);
void joint_states_callback(const sensor_msgs::JointState &msg);
void control_mode_callback(const std_msgs::String& msg);
void left_hand_pose_callback(const geometry_msgs::Pose& pose_msg);
void right_hand_pose_callback(const geometry_msgs::Pose& pose_msg);
void update_JntArray(KDL::JntArray& l_arm, KDL::JntArray& r_arm);
void KDL_to_arma(const KDL::Jacobian& KDL_jac, arma::mat& arma_jac);
void KDL_to_arma(const KDL::Twist& KDL_twist, arma::vec& arma_twist);
void KDL_to_arma(const KDL::Frame& frame, arma::vec& pose_vec);
void KDL_to_arma(const KDL::Vector& pos, const KDL::Rotation& orient, arma::vec& pose_vec);
void arma_to_KDL(const arma::vec& arma_jnt, KDL::JntArray& jnt_q);
void arma_to_KDL(const arma::vec& pose, KDL::Vector& pos, KDL::Rotation& rot);
void pose_msg_to_arma(const geometry_msgs::Pose& pose_msg, arma::vec& pose_vec);

#endif
