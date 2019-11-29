
#include <yumi_test_controllers.h>
#include <vector>

sensor_msgs::JointState joints_state;

std_msgs::Float64 left_command;
std_msgs::Float64 right_command;
std_msgs::Float64 left_joint_pos;
std_msgs::Float64 right_joint_pos;
std_msgs::Float64 pos_val;

int num_joints = 14;
int num_joints_arm = 7;
int test_joint_number = 2;
int left_joint_state_idx = 4;
int right_joint_state_idx = 4;

vector<float>  q = {0,0,0,0,0,0,0};
vector<float> values= {0,0,0,0,0,2.5,0};

auto last_sample_time = std::chrono::high_resolution_clock::now();
auto last_call_time = std::chrono::high_resolution_clock::now();
auto curr_call_time = std::chrono::high_resolution_clock::now();;

double sine_period = 10.0;
double sine_freq = 1 / sine_period;
double sine_amp = 0.3;

double sampling_freq = 200;
double sampling_period = 1/sampling_freq;

ros::Publisher left_controller_pub;
ros::Publisher right_controller_pub;
ros::Publisher left_state_pub;
ros::Publisher right_state_pub;
ros::Publisher pos_signal_pub;
ros::Subscriber sub;

bool g_quit;
void quitRequested(int sig)
{
	g_quit = true;
	left_command.data = 0.0;
	right_command.data = 0.0;

	left_controller_pub.publish(left_command);
	right_controller_pub.publish(right_command);

	ros::shutdown();
}

void send_pos_joints()
{
	curr_call_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> sine_elapsed = curr_call_time - last_call_time;

	std::chrono::duration<double> sampling_elapsed = curr_call_time - last_sample_time;

	if(sampling_elapsed.count() > sampling_period)
	{
		for (int i=0; i < q.size(); ++i)
		{
			pos_val.data = q[i];  //for first loop we get the the angle for joint 1 etc
			cout << i << pos_val.data << endl;
			pos_signal_pub.publish(pos_val);
			//cout << "Time since last joint state received = " << sine_elapsed.count() << endl;
			// cout << "Sine value = " << pos_val << endl;

			left_command.data = pos_val.data;

			right_command.data = pos_val.data;
			//cout << "here #1" << endl;
			// cout << "Publishing command" << endl;
			left_controller_pub.publish(left_command);
			// right_controller_pub.publish(right_command); WHY HAS THIS BEEN //'d

			last_sample_time = std::chrono::high_resolution_clock::now();
		}
	}

	if(sine_elapsed.count() > sine_period)
	{
		last_call_time = std::chrono::high_resolution_clock::now();
		cout << "here #2" << endl;
	}
}



void joint_states_callback(const sensor_msgs::JointState &msg)
{
	// ROS_INFO("Joint states update!");
	joints_state.name = msg.name;
	joints_state.position = msg.position;
	joints_state.velocity = msg.velocity;
	joints_state.effort = msg.effort;

	left_joint_pos.data = joints_state.position[left_joint_state_idx];
	right_joint_pos.data = joints_state.position[right_joint_state_idx];

	left_state_pub.publish(left_joint_pos);
	right_state_pub.publish(right_joint_pos);
}


void addValues(vector<float>& q,vector<float> angleValues)
{
	for (int i=0; i<7; ++i)
	{	
		q.pop_back();
	}
	//cout << "im here #3 " << endl;
	for (int j=0; j<7; ++j)
	{
		q.push_back (angleValues[j]);
	}
}




int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "test_joint_vel_control");
	ros::NodeHandle nh;

    sub = nh.subscribe("/yumi/joint_states", 1000, joint_states_callback);

	signal(SIGTERM, quitRequested);
	signal(SIGINT, quitRequested);
	signal(SIGHUP, quitRequested);

	pos_signal_pub = nh.advertise<std_msgs::Float64>("/yumi/pos_signal", 1000);
    left_controller_pub = nh.advertise<std_msgs::Float64>("/yumi/joint_pos_controller_" + std::to_string(test_joint_number) + "_l/command", 1000);
	right_controller_pub = nh.advertise<std_msgs::Float64>("/yumi/joint_pos_controller_" + std::to_string(test_joint_number) + "_r/command", 1000);

    left_state_pub = nh.advertise<std_msgs::Float64>("/yumi/joint_" + std::to_string(test_joint_number) + "_l/state", 1000);
	right_state_pub = nh.advertise<std_msgs::Float64>("/yumi/joint_" + std::to_string(test_joint_number) + "_r/state", 1000);

	while( !g_quit && ros::ok() )
  	{
		ros::spinOnce();
		addValues(q, values);
		send_pos_joints();
	}

	return EXIT_SUCCESS;
}

