// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

// KDL
#include <kdl/kdl.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <armadillo>

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

void print_joint_values(const KDL::JntArray& q)
{
	std::cout << "[";
	for(uint i=0;i<num_joints_arm;i++)
		std::cout << q(i) << " ";
	std::cout << "]" << std::endl;
}

void print_joint_values(const arma::vec& q)
{
	std::cout << "[";
	for(uint i=0;i<num_joints_arm;i++)
		std::cout << q(i) << " ";
	std::cout << "]" << std::endl;
}

void print_Jacobian(KDL::Jacobian J)
{
	for(uint i=0;i<J.rows();i++)
	{
		for(uint j=0;j<J.columns();j++)
			std::cout << " " << J(i,j);
		std::cout << std::endl;
	}
	std::cout << std::endl;
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

void rearrange_YuMi_joints(std::vector<std_msgs::Float64>& joint_pos)
{
	double tmp_joint;
	tmp_joint = joint_pos[6].data;
	for(uint i=6;i>2;i--)
		joint_pos[i].data = joint_pos[i-1].data;
	joint_pos[2].data = tmp_joint;
}

void joint_states_callback(const sensor_msgs::JointState &msg)
{
	// ROS_INFO("Joint states update!");
	joints_state.name = msg.name;
	joints_state.position = msg.position;
	joints_state.velocity = msg.velocity;
	joints_state.effort = msg.effort;

	for(uint i=0;i<num_joints_arm;i++)
	{
		left_joint_pos[i].data = joints_state.position[2*(i+1)];
		right_joint_pos[i].data = joints_state.position[2*(i+1)+1];
	}

	rearrange_YuMi_joints(left_joint_pos);
	rearrange_YuMi_joints(right_joint_pos);
}

void joint_solver_callback(const std_msgs::String& msg)
{
	joint_solver = msg.data;
	std::cout << "Switched control mode to "<< joint_solver;
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