#include "yumi_test_controllers.h"

int main( int argc, char* argv[] )
{
    if(argc!=2)
    {
        std::cout << "ERROR: specify \"position\" or \"velocity\" control, e.g. 'test_IK_control position' (nargc = " << argc << ")"
                  << std::endl;
        return -1;
    }
    /*else if(argv[1]!="position")// || argv[1]!="velocity")
    {
        std::cout << "ERROR: specify \"position\" or \"velocity\" control, e.g. 'test_IK_control position'"
                  << std::endl;
        return -1;
    }*/

    control_mode = argv[1];
	ros::init(argc, argv, "test_joint_" + control_mode + "_vel_control");
	ros::NodeHandle nh;

	// Subscribe ROS nodes
	sub = nh.subscribe("/yumi/joint_states", 1000, joint_states_callback);
	sub_joint_solver = nh.subscribe("/yumi/joint_solver", 1000, joint_solver_callback);

	// Initialize publishers
	for(uint i=0;i<num_joints_arm;i++)
	{
	    if(control_mode=="position") {
            left_controller_pub.at(i) = nh.advertise<std_msgs::Float64>(
                    "/yumi/joint_pos_controller_" + std::to_string(i + 1) + "_l/command", 1000);
            right_controller_pub.at(i) = nh.advertise<std_msgs::Float64>(
                    "/yumi/joint_pos_controller_" + std::to_string(i + 1) + "_r/command", 1000);
        }
        else if(control_mode=="velocity") {
            left_controller_pub.at(i) = nh.advertise<std_msgs::Float64>(
                    "/yumi/joint_vel_controller_" + std::to_string(i + 1) + "_l/command", 1000);
            right_controller_pub.at(i) = nh.advertise<std_msgs::Float64>(
                    "/yumi/joint_vel_controller_" + std::to_string(i + 1) + "_r/command", 1000);
        }

	}
	
	signal(SIGTERM, quitRequested);
	signal(SIGINT, quitRequested);
	signal(SIGHUP, quitRequested);

	// Initialize a KDL tree from urdf file
	KDL::Tree yumi_tree;
	if (!kdl_parser::treeFromFile("/home/jsilverio/yumi_depends_ws/devel/yumi.urdf", yumi_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	std::cout << "Number of joints in tree: " << yumi_tree.getNrOfJoints() << std::endl;
	std::cout << "Number of segments in tree: " << yumi_tree.getNrOfSegments() << std::endl;

	// Initialize chains
	KDL::Chain left_arm_chain, right_arm_chain;
	yumi_tree.getChain("yumi_body","yumi_link_7_l",left_arm_chain);
	yumi_tree.getChain("yumi_body","yumi_link_7_r",right_arm_chain);		

	std::cout << "Number of joints in left arm chain: " << left_arm_chain.getNrOfJoints() << std::endl;
	std::cout << "Number of joints in right arm chain: " << right_arm_chain.getNrOfJoints() << std::endl;

    // Forward kinematics "solver" (why a solver?)
    KDL::ChainFkSolverPos_recursive fk_left = KDL::ChainFkSolverPos_recursive(left_arm_chain);
    KDL::ChainFkSolverPos_recursive fk_right = KDL::ChainFkSolverPos_recursive(right_arm_chain);

	// Inverse kinematics solver (just to get the Jacobian matrix)
	KDL::ChainJntToJacSolver jnt_to_jac_solver_left(left_arm_chain);		
	KDL::ChainJntToJacSolver jnt_to_jac_solver_right(right_arm_chain);		
	KDL::ChainIkSolverVel_pinv pinv_solver_left(left_arm_chain);
	KDL::ChainIkSolverVel_pinv pinv_solver_right(right_arm_chain);

	// Initialize joint vectors
	KDL::JntArray joints_l_arm = KDL::JntArray(left_arm_chain.getNrOfJoints());
	KDL::JntArray joints_r_arm = KDL::JntArray(right_arm_chain.getNrOfJoints());
	KDL::JntArray u_left = KDL::JntArray(left_arm_chain.getNrOfJoints());
	KDL::JntArray u_right = KDL::JntArray(right_arm_chain.getNrOfJoints());

	// Initialize KDL Jacobians
	KDL::Jacobian J_left;
	KDL::Jacobian J_right;
	J_left.resize(left_arm_chain.getNrOfJoints());
	J_right.resize(right_arm_chain.getNrOfJoints());
    for(uint i=0; i< num_joints_arm; i++)
            q_homing_left(i) = q_homing_init[i];
    q_homing_right = q_homing_left;
    q_homing_right(0) = -q_homing_right(0);          // two joints need flipping
    q_homing_right(2) = -q_homing_right(2);

	// Initialize Armadillo Jacobians
	arma::mat arma_J_left;
	arma::mat arma_J_right;
	arma_J_left.zeros(6,num_joints_arm);
	arma_J_right.zeros(6,num_joints_arm);

	// Initalize arma IK objects
	arma::mat pinv_J_left;
	arma::mat pinv_J_right;
	pinv_J_left.zeros(6,6);
	pinv_J_right.zeros(6,6);
	arma::vec arma_u_left;
	arma::vec arma_u_right;
	arma_u_left.zeros(num_joints_arm);
	arma_u_right.zeros(num_joints_arm);
	double rcond = 1E-12;	// regularize pseudoinverse

	// KDL pose
	KDL::Frame pose_left;
	KDL::Frame pose_right;
	KDL::Vector desired_pos_left(0.2,0.5,0.6);
	KDL::Vector desired_pos_right(0.2,-0.5,0.6);
	KDL::Rotation rot_left_desired;		// initialized with identity
	KDL::Rotation rot_right_desired;	
	KDL::Rotation tmp;
//	rot_left_desired.DoRotY(2.0);
//	rot_right_desired.DoRotY(1.5);
//	rot_right_desired.DoRotZ(1.5);
	
	// KDL twists (for the desired task space commands)
	KDL::Vector v_l(0.0,0.0,0.0);
	KDL::Vector v_r(0.0,0.0,0.0);
	KDL::Vector rot_l(0.0,0.0,0.0);
	KDL::Vector rot_r(0.0,0.0,0.0);
	KDL::Twist twist_left(v_l,rot_l);
	KDL::Twist twist_right(v_r,rot_r);
	arma::vec arma_twist_left(6);
	arma::vec arma_twist_right(6);

	// Initialize rate, sleep and spin once
	ros::Rate rate(50); // 50 hz update rate
	rate.sleep();
	ros::spinOnce();	// if I don't sleep AND spin once, all joints will be read 0. Why? 

	while( !g_quit && ros::ok() )
  	{
		// -- Get Jacobian
		// Read joint angles
		update_JntArray(joints_l_arm,joints_r_arm);
		jnt_to_jac_solver_left.JntToJac(joints_l_arm,J_left);
		jnt_to_jac_solver_right.JntToJac(joints_r_arm,J_right);
		//print_Jacobian(J_left); // print Jacobian
		//print_Jacobian(J_right); // print Jacobian
		KDL_to_arma(J_left,arma_J_left);
		KDL_to_arma(J_right,arma_J_right);
		//arma_J_left.print("left Jac:");
		//arma_J_right.print("right Jac:");

		// -- Compute error in task space
		fk_left.JntToCart(joints_l_arm,pose_left);
		fk_right.JntToCart(joints_r_arm,pose_right);

		// left end-effector
		twist_left.vel = desired_pos_left - pose_left.p;
		tmp = pose_left.M;
		tmp.SetInverse();		
		tmp = rot_left_desired*tmp;
		twist_left.rot = tmp.GetRot();
		KDL_to_arma(twist_left,arma_twist_left);

        // right end-effector
		twist_right.vel = desired_pos_right - pose_right.p;
		tmp = pose_right.M;
		tmp.SetInverse();		
		tmp = rot_right_desired*tmp;
		twist_right.rot = tmp.GetRot();
		KDL_to_arma(twist_right,arma_twist_right);
//		std::cout << "Left Hand:" << pose_left.p.data[0] <<" " << pose_left.p.data[1] <<" " << pose_left.p.data[2] << " " << std::endl;
//		std::cout << "Right Hand:" << pose_right.p.data[0] <<" " << pose_right.p.data[1] <<" " << pose_right.p.data[2] <<" " << std::endl;

		// -- Generate joint commands with KDL solver
		if(joint_solver=="KDL solver")
		{
			pinv_solver_left.CartToJnt(joints_l_arm,twist_left,u_left);
			pinv_solver_right.CartToJnt(joints_r_arm,twist_right,u_right);
		}
		// -- Generate joint commands with my pseudo-inverse
		else if(joint_solver=="pinv")
		{
			pinv_J_left  = arma::solve(arma_J_left.t()*arma_J_left + arma::eye(num_joints_arm,num_joints_arm)*rcond,arma_J_left.t());	// A\B
			pinv_J_right = arma::solve(arma_J_right.t()*arma_J_right + arma::eye(num_joints_arm,num_joints_arm)*rcond,arma_J_right.t());
			arma_u_left = pinv_J_left*arma_twist_left;
			arma_u_right = pinv_J_right*arma_twist_right;
			arma_to_KDL(arma_u_left,u_left);	
			arma_to_KDL(arma_u_right,u_right);	
		}
		// -- Generate joint homing commands
		else if(joint_solver=="homing")
		{
			KDL::Subtract(q_homing_left,joints_l_arm,u_left);
			KDL::Subtract(q_homing_right,joints_r_arm,u_right);
		}

		// -- Prepare for publishing
		for(uint i=0;i<num_joints_arm;i++)
		{
            if(control_mode=="velocity")
            {
                left_command.at(i).data = u_left(i);
                right_command.at(i).data = u_right(i);
            }
            if(control_mode=="position")
            {
                left_command.at(i).data = u_left(i) * 0.1 + joints_l_arm(i);
                right_command.at(i).data = u_right(i) * 0.1 + joints_r_arm(i);
            }
		}

		// -- Send joint commands to robot
		publish_commands();

		// -- ROS spin and sleep
		ros::spinOnce();
		rate.sleep();
	}

	return EXIT_SUCCESS;
}