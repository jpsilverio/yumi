// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

// KDL
#include <kdl/kdl.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <armadillo>

// Useful defines
#define DQ_MAX 360*M_PI/180       // maximum joint angular velocity

// Reminder: declare global variables here as 'extern' instead of including the header -- that would result in
// multiple declaration errors when linking
extern int num_joints_arm;
extern std::vector<std_msgs::Float64> left_command;
extern std::vector<std_msgs::Float64> right_command;
extern std::vector<ros::Publisher> left_controller_pub;
extern std::vector<ros::Publisher> right_controller_pub;
extern bool g_quit;
extern sensor_msgs::JointState joints_state;
extern std::vector<std_msgs::Float64> left_joint_pos;
extern std::vector<std_msgs::Float64> right_joint_pos;
extern std::string joint_solver;        // to choose what IK solver to use or joint space tracking instead
extern std::string control_mode;        // position or velocity
extern KDL::Vector desired_pos_left;
extern KDL::Vector desired_pos_right;
extern KDL::Rotation rot_left_desired;		// the object is initialized with identity
extern KDL::Rotation rot_right_desired;
extern geometry_msgs::Pose left_hand_desired_pose;
extern geometry_msgs::Pose right_hand_desired_pose;

using namespace std;

void publish_commands()
{
	// The urdf model has joint 7 right after joint 2 and the rest shifted forward
	for(uint i=0;i<2;i++)
	{
		left_controller_pub.at(i).publish(left_command.at(i));
		right_controller_pub.at(i).publish(right_command.at(i));
	}
	for(uint i=2;i<num_joints_arm-1;i++)
	{
		left_controller_pub.at(i).publish(left_command.at(i+1));
		right_controller_pub.at(i).publish(right_command.at(i+1));
	}
	left_controller_pub.at(num_joints_arm-1).publish(left_command.at(2));
	right_controller_pub.at(num_joints_arm-1).publish(right_command.at(2));
}

void rearrange_YuMi_joints(std::vector<std_msgs::Float64>& joint_pos)
{
    double tmp_joint;
    tmp_joint = joint_pos[6].data;
    for(uint i=6;i>2;i--)
        joint_pos[i].data = joint_pos[i-1].data;
    joint_pos[2].data = tmp_joint;
}

void pose_msg_to_KDL(const geometry_msgs::Pose& pose_msg, KDL::Vector& pos, KDL::Rotation& rot)
{
    pos(0) = pose_msg.position.x;
    pos(1) = pose_msg.position.y;
    pos(2) = pose_msg.position.z;
    rot = KDL::Rotation::Quaternion(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);
    //std::cout<<"new pose set"<<std::endl;
}

// -- Subscriber callbacks
void joint_states_callback(const sensor_msgs::JointState &msg)
{
    /*joints_state.name = msg.name;
    joints_state.position = msg.position;
    joints_state.velocity = msg.velocity;
    joints_state.effort = msg.effort;*/

    for(uint i=0;i<num_joints_arm;i++)
    {
        /*left_joint_pos[i].data = joints_state.position[2*(i+1)];
        right_joint_pos[i].data = joints_state.position[2*(i+1)+1];*/
        left_joint_pos[i].data = msg.position[2*(i+1)];
        right_joint_pos[i].data = msg.position[2*(i+1)+1];
    }

    rearrange_YuMi_joints(left_joint_pos);
    rearrange_YuMi_joints(right_joint_pos);
}

void joint_solver_callback(const std_msgs::String& msg)
{
    joint_solver = msg.data;
    cout << "Switched control mode to "<< joint_solver;
}

void left_pose_callback(const geometry_msgs::Pose& pose_msg)
{
    //ROS_INFO("Left Hand pose update!");
    pose_msg_to_KDL(pose_msg, desired_pos_left, rot_left_desired);
}

void right_pose_callback(const geometry_msgs::Pose& pose_msg)
{
    //ROS_INFO("Right Hand pose update!");
    pose_msg_to_KDL(pose_msg, desired_pos_right, rot_right_desired);
}

// -- Printing stuff to stdout
void print_joint_values(const KDL::JntArray& q)
{
	cout << "[";
	for(uint i=0;i<num_joints_arm;i++)
		cout << q(i) << " ";
	cout << "]" << endl;
}

void print_joint_values(const arma::vec& q)
{
	cout << "[";
	for(uint i=0;i<num_joints_arm;i++)
		cout << q(i) << " ";
	cout << "]" << endl;
}

void print_Jacobian(KDL::Jacobian J)
{
	for(uint i=0;i<J.rows();i++)
	{
		for(uint j=0;j<J.columns();j++)
			cout << " " << J(i,j);
		cout << endl;
	}
	cout << endl;
}

void quitRequested(int sig)
{
	g_quit = true;
	for(uint i=0;i<num_joints_arm;i++)
	{
        if(control_mode=="position")
        {
            left_command.at(i).data = left_joint_pos[i].data;
            right_command.at(i).data = right_joint_pos[i].data;
        }
        else if(control_mode=="velocity")
        {
            left_command.at(i).data = 0.0;
            right_command.at(i).data = 0.0;
        }
	}
	publish_commands();
	ros::shutdown();
}

void saturate(KDL::JntArray& dq)
{
    for(uint i=0; i<num_joints_arm ;i++)
    {
        //cout << i << " " << dq(i) << endl;
        dq(i) = fabs(dq(i)) > DQ_MAX ? (dq(i) > 0 ? DQ_MAX : -DQ_MAX) : dq(i);
    }
}

void update_JntArray(KDL::JntArray& l_arm, KDL::JntArray& r_arm)
{
	for(int i=0;i<num_joints_arm;i++){
		l_arm(i) = left_joint_pos[i].data;
		r_arm(i) = right_joint_pos[i].data;
	}
}

void KDL_to_arma(const KDL::Jacobian& KDL_jac, arma::mat& arma_jac)
{
	for(unsigned int i = 0; i<KDL_jac.rows(); i++){
		for(unsigned int j = 0; j<KDL_jac.columns(); j++){
			arma_jac(i,j) = KDL_jac(i,j);
		}
	}
}

void KDL_to_arma(const KDL::Twist& KDL_twist, arma::vec& arma_twist)
{
	for(unsigned int i=0; i<6; i++)
		arma_twist(i) = KDL_twist(i);
}

void arma_to_KDL(const arma::vec& arma_jnt, KDL::JntArray& jnt_q)
{
	for(unsigned int i=0; i<num_joints_arm; i++)
		jnt_q(i) = arma_jnt(i);
}