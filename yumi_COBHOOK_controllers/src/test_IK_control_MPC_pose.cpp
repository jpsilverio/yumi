/*
*	The robot tracks a pose trajectory generated using MPC. Uses
*	João Silvério, 2019
*/
#include "yumi_test_controllers.h"

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "test_joint_vel_control");
    ros::NodeHandle nh;

    // Subscribe ROS nodes
    sub = nh.subscribe("/yumi/joint_states", 1000, joint_states_callback);
    sub_control_mode = nh.subscribe("/yumi/control_mode", 1000, control_mode_callback);
    sub_left_pose = nh.subscribe("/yumi/control_left_hand_pose",1000,left_hand_pose_callback);
    sub_right_pose = nh.subscribe("/yumi/control_right_hand_pose",1000,right_hand_pose_callback);

    // Initialize publishers
    for(uint i=0;i<num_joints_arm;i++)
    {
        left_controller_pub.at(i) = nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_" + std::to_string(i+1) + "_l/command", 1000);
        right_controller_pub.at(i) = nh.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_" + std::to_string(i+1) + "_r/command", 1000);
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
    KDL::SetToZero(joints_l_arm);
    KDL::SetToZero(joints_r_arm);
    KDL::SetToZero(u_left);
    KDL::SetToZero(u_right);

    // Initialize KDL Jacobians
    KDL::Jacobian J_left;
    KDL::Jacobian J_right;
    J_left.resize(left_arm_chain.getNrOfJoints());
    J_right.resize(right_arm_chain.getNrOfJoints());
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
    KDL::Rotation tmp;
//	desired_rot_left.DoRotY(2.0);
//	desired_rot_right.DoRotY(1.5);
//	desired_rot_right.DoRotZ(1.5);
    arma::vec arma_pose_left(7);
    arma::vec arma_pose_right(7);

    // KDL twist
    KDL::Vector v_l(0.0,0.0,0.0);
    KDL::Vector v_r(0.0,0.0,0.0);
    KDL::Vector rot_l(0.0,0.0,0.0);
    KDL::Vector rot_r(0.0,0.0,0.0);
    KDL::Twist twist_left(v_l,rot_l);
    KDL::Twist twist_right(v_r,rot_r);
    arma::vec arma_twist_left(6);
    arma::vec arma_twist_right(6);

    // MPC
    int nbData = 1000; //Number of datapoints
    int nbVarTan = 3 + 3; //Dimension of tangent space data (here: TE3 + TS3)
    int nbDeriv = 2; //Number of static & dynamic features (D=2 for [x,dx])
    int nbVar = nbVarTan * nbDeriv; //Dimension of tangent state space
    int nbVarMan = nbVarTan + 1; //Dimension of the manifold (here: E3 + S3)
    double params_diagRegFact = 1E-4; //Regularization of covariance
    double dt = 1E-2; //Time step duration
    double rfactor = 5E1;	//Control cost in LQR

    //Control cost matrix
    arma::mat R = arma::eye(nbVarTan, nbVarTan) * rfactor;
    //R.submat(0, 0, 2, 2)=diagmat(ones(3,1)*1E-1); % Set different position control cost

    //Target and desired covariance
    KDL_to_arma(desired_pos_left,desired_rot_left,xTar);
    arma::mat uCov = arma::eye(nbVarTan, nbVarTan) * 1E-3;

    //Eigendecomposition
    arma::vec tmpEigenValues;
    arma::mat tmpEigenVectors;
    arma::eig_sym(tmpEigenValues, tmpEigenVectors, uCov);
    arma::mat U0 = tmpEigenVectors * diagmat(pow(tmpEigenValues, 0.5));

    arma::mat A = arma::eye(nbVarTan * 2, nbVarTan * 2);
    A.submat(0, nbVarTan, nbVarTan - 1, (nbVarTan * 2) - 1) = arma::eye(nbVarTan, nbVarTan) * dt;
    arma::mat B = join_vert(arma::eye(nbVarTan, nbVarTan) * pow(dt, 2), arma::eye(nbVarTan, nbVarTan) * dt);

    arma::colvec duTar = arma::zeros(nbVarTan * (nbDeriv - 1), 1);
    arma::vec x_old = arma_pose_left;
    arma::vec U = arma::zeros(nbVar, 1);

    arma::mat Ac, U1, uCovTmp;
    arma::mat Q = arma::zeros(nbVar, nbVar);
    arma::mat P, L;
    arma::vec ddu;

    // Homing joint references
    KDL::JntArray q_homing_left = KDL::JntArray(left_arm_chain.getNrOfJoints());
    KDL::JntArray q_homing_right = KDL::JntArray(right_arm_chain.getNrOfJoints());

    // Initialize rate, sleep and spin once
    ros::Rate rate(50); // 50 hz update rate
    rate.sleep();
    ros::spinOnce();	// if I don't sleep AND spin once, all joints will be read 0. Why?

    // Initialize homing joints
    update_JntArray(joints_l_arm,joints_r_arm);
    q_homing_left = joints_l_arm;
    q_homing_right = joints_r_arm;
    //std::cout<<"Q homing left: "; print_joint_values(q_homing_left);
    //std::cout<<"Q homing right: "; print_joint_values(q_homing_right);

    // Control mode sanity check
    if(control_mode != "")
    {
        std::cout << std::endl << "ERROR: Set control_mode=\"\" in the declaration, compile and run again." << std::endl<< std::endl;
        return -1;
    }

    // -- Obtain reference trajectory from MPC (only left hand for now)
    fk_left.JntToCart(joints_l_arm,pose_left);
    fk_right.JntToCart(joints_r_arm,pose_right);
    KDL_to_arma(pose_left,arma_pose_left);
    KDL_to_arma(pose_right,arma_pose_right);

    std::vector<arma::vec> targets(nbData);
    uint n = 0;

    while( !g_quit && ros::ok() )
    {
        if(restart_plan){
            x = arma_pose_left;
            x_old = x;

            for (int t = 0; t < nbData; t++) {
//                xTar.print("xTar:");
                U.subvec(0, nbVarTan - 1) = arma::zeros(nbVarTan, 1); //Set tangent space at x
                //Transportation of velocity vector from x_old to x
                if (norm(x_old - x, 2) != 0) {					// haven't reached target
                    Ac = transp(x_old, x);
                } else {
                    Ac = eye(nbVarTan, nbVarTan);
                }
                U.subvec(nbVarTan, (2 * nbVarTan) - 1) = Ac * U.subvec(nbVarTan, (2 * nbVarTan) - 1); //Transport of velocity
                U.subvec(0, 2) = x.subvec(0, 2); // Transport of position (action part)

                //Transportation of uCov from xTar to x
                Ac = transp(xTar, x);
                if (norm(xTar - x, 2) != 0) {					// haven't reached target
                    Ac = transp(xTar, x);
                } else {
                    Ac = eye(nbVarTan, nbVarTan);
                }
                U1 = Ac * U0;
                uCovTmp = U1 * U1.t();
                Q = zeros(nbVar, nbVar);
                Q.submat(0, 0, nbVarTan - 1, nbVarTan - 1) = inv(uCovTmp); //Precision matrix
                P = solveAlgebraicRiccati_eig_discrete(A, B, Q, R); // * (inv(R) * B.t()), (Q + Q.t()) / 2);

                L = inv(B.t() * P * B + R) * B.t() * P * A; //Feedback gain (discrete version)
                ddu = L * (join_vert(logmap(xTar, x), duTar) - U); //Compute acceleration
                U = A * U + B * ddu; //Update U
                x_old = x;
                x = expmap(U.subvec(0, nbVarTan - 1), x);
                targets.at(t) = x;
            }
            restart_plan = false;
            std::cout<<"restarted"<<std::endl;
            n = 0;
        }

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

        // -- Forward kinematics
        fk_left.JntToCart(joints_l_arm,pose_left);
        fk_right.JntToCart(joints_r_arm,pose_right);
        KDL_to_arma(pose_left,arma_pose_left);
        KDL_to_arma(pose_right,arma_pose_right);

        // -- Compute error in task space
        arma_to_KDL(targets.at(n),desired_pos_left,desired_rot_left);
        twist_left.vel = desired_pos_left - pose_left.p;
        tmp = pose_left.M;
        tmp.SetInverse();
        tmp = desired_rot_left*tmp;
        twist_left.rot = tmp.GetRot();
        KDL_to_arma(twist_left,arma_twist_left);

        twist_right.vel = desired_pos_right - pose_right.p;
        tmp = pose_right.M;
        tmp.SetInverse();
        tmp = desired_rot_right*tmp;
        twist_right.rot = tmp.GetRot();
        KDL_to_arma(twist_right,arma_twist_right);
//		std::cout << "Left Hand:" << pose_left.p.data[0] <<" " << pose_left.p.data[1] <<" " << pose_left.p.data[2] << " " << std::endl;
//		std::cout << "Right Hand:" << pose_right.p.data[0] <<" " << pose_right.p.data[1] <<" " << pose_right.p.data[2] <<" " << std::endl;

        // -- Generate joint commands with KDL solver
        if(control_mode=="IK-solver")
        {
            pinv_solver_left.CartToJnt(joints_l_arm,twist_left,u_left);
            pinv_solver_right.CartToJnt(joints_r_arm,twist_right,u_right);
            if(n<nbData-1)
                n++;
        }

            // -- Generate joint commands with my pseudo-inverse
        else if(control_mode=="pinv")
        {
            pinv_J_left  = arma::solve(arma_J_left.t()*arma_J_left + arma::eye(num_joints_arm,num_joints_arm)*rcond,arma_J_left.t());	// A\B
            pinv_J_right = arma::solve(arma_J_right.t()*arma_J_right + arma::eye(num_joints_arm,num_joints_arm)*rcond,arma_J_right.t());
            arma_u_left = pinv_J_left*arma_twist_left;
            arma_u_right = pinv_J_right*arma_twist_right;
            arma_to_KDL(arma_u_left,u_left);
            arma_to_KDL(arma_u_right,u_right);
            if(n<nbData-1)
                n++;
        }
            // -- Generate joint homing commands
        else if(control_mode=="homing")
        {
            KDL::Subtract(q_homing_left,joints_l_arm,u_left);
            KDL::Subtract(q_homing_right,joints_r_arm,u_right);
        }

        // -- Prepare for publishing
        for(uint i=0;i<num_joints_arm;i++)
        {
            left_command.at(i).data = u_left(i);
            right_command.at(i).data = u_right(i);
        }

        // -- Send joint commands to robot
        publish_commands();

        // spin and sleep
        ros::spinOnce();
        rate.sleep();

    }

    return EXIT_SUCCESS;
}

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
        left_command.at(i).data = 0.0;
        right_command.at(i).data = 0.0;
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

void left_hand_pose_callback(const geometry_msgs::Pose& pose_msg)
{
    ROS_INFO("Left Hand pose update!");
    left_hand_desired_pose.position = pose_msg.position;
    left_hand_desired_pose.orientation = pose_msg.orientation;

    std::cout << "Left hand position: " << left_hand_desired_pose.position << std::endl;
    std::cout << "Left hand Orientation: " << left_hand_desired_pose.orientation << std::endl;
    restart_plan = true;
    pose_msg_to_arma(pose_msg, xTar);
}

void right_hand_pose_callback(const geometry_msgs::Pose& pose_msg)
{
    ROS_INFO("Right Hand pose update!");
    right_hand_desired_pose.position = pose_msg.position;
    right_hand_desired_pose.orientation = pose_msg.orientation;

    std::cout << "Right hand position: " << right_hand_desired_pose.position << std::endl;
    std::cout << "Right hand Orientation: " << right_hand_desired_pose.orientation << std::endl;
    restart_plan = true;
    pose_msg_to_arma(pose_msg, xTar);
}

void control_mode_callback(const std_msgs::String& msg)
{
    control_mode = msg.data;
    std::cout << "Switched control mode to "<< control_mode;
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

void KDL_to_arma(const KDL::Frame& frame, arma::vec& pose_vec)
{
    KDL_to_arma(frame.p,frame.M,pose_vec);
}

void KDL_to_arma(const KDL::Twist& KDL_twist, arma::vec& arma_twist)
{
    for(unsigned int i=0; i<6; i++)
        arma_twist(i) = KDL_twist(i);
}

void KDL_to_arma(const KDL::Vector& pos, const KDL::Rotation& orient, arma::vec& pose_vec)
{
    for(unsigned int i=0; i<3; i++)
        pose_vec(i) = pos(i);
    orient.GetQuaternion(pose_vec(4),pose_vec(5),pose_vec(6),pose_vec(3));
}

void pose_msg_to_arma(const geometry_msgs::Pose& pose_msg, arma::vec& pose_vec)
{
    pose_vec(0) = pose_msg.position.x;
    pose_vec(1) = pose_msg.position.y;
    pose_vec(2) = pose_msg.position.z;
    pose_vec(3) = pose_msg.orientation.w;
    pose_vec(4) = pose_msg.orientation.x;
    pose_vec(5) = pose_msg.orientation.y;
    pose_vec(6) = pose_msg.orientation.z;
    std::cout<<"new pose set"<<std::endl;
}

void arma_to_KDL(const arma::vec& arma_jnt, KDL::JntArray& jnt_q)
{
    for(unsigned int i=0; i<num_joints_arm; i++)
        jnt_q(i) = arma_jnt(i);
}

void arma_to_KDL(const arma::vec& pose, KDL::Vector& pos, KDL::Rotation& rot)
{
    for(unsigned int i=0; i<3; i++)
        pos(i) = pose(i);
    rot = KDL::Rotation::Quaternion(pose(4),pose(5),pose(6),pose(3));
}
