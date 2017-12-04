#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

// ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "Pose.h"
#include "TargetState.h"

// Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"

#include <signal.h>
#include <math.h>

#include <iostream>
#include <string>
#include <fstream>


using namespace std;



//Defining linear velocity details

#define LINEAR_VELOCITY 0.3

// state machine states
#define STATE_MACHINE_TRANSLATE 0
#define STATE_MACHINE_TEMP 1
#define STATE_MACHINE_GO_HOME 2 
#define STATE_STOP_TIME_UP 3
int state_machine_state = STATE_MACHINE_TRANSLATE;
// Random number generator
random_numbers::RandomNumberGenerator *rng;

typedef struct{
float x;
float y;
float theta;
}pro_pose;

typedef struct
{
	string name;
	int pro_target_id;
	int pro_target_collected_data;
}pro_rover_target_data;



string rover_name;
char host[128];
bool is_published_name = false;


int simulation_mode = 0;
float mobility_loop_time_step = 0.1;
float status_publish_interval = 5;
float kill_switch_timeout = 10;

pose current_location;

int transitions_to_auto = 0;
double time_stamp_transition_to_auto = 0.0;
double pro_time_stamp_target_collected = 0.0;
double temp_time_stamp = 0.0;
//double time_stamp_transition_to_auto = 0.0;

//Adding for Project Purpose
bool pro_targetdetected =false;
bool pro_target_collected = false;
bool pro_targets_collected[256]={false};
bool pro_start_home_search=false;
bool pro_start_target_search=true;
int pro_count_target_collected=0;
int pro_count_total_collected=0;
float pro_dist_total=0;
float pro_dist_mid=0;
float pro_count_theta=0.0;
float temp_theta=0.0;
float pro_goal_theta_temp=0.0;
pro_rover_target_data rovers1[6]={{"ajax",0,0},{"achilles",0,0},{"aeneas",0,0},{"diomedes",0,0},{"hector",0,0},{"paris",0,0}};
pro_pose pro_goal_location;

ofstream outputFile;

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher target_collected_publisher;
ros::Publisher angular_publisher;
ros::Publisher messagePublish;
ros::Publisher posePublish;
ros::Publisher global_average_headingPublish;
ros::Publisher local_average_headingPublish;
ros::Publisher debug_publisher;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber poseSubscriber;
ros::Subscriber global_average_headingSubscriber;
ros::Subscriber local_average_headingSubscriber;
ros::Subscriber target_collectedSubscriber;
ros::Subscriber messageSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

//target data processing 
void process_home_tags(const shared_messages::TagsImage::ConstPtr &message); 
void process_resources(const shared_messages::TagsImage::ConstPtr &message); 

// Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

// OS Signal Handler
void sigintEventHandler(int signal);

// Callback handlers
void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message);
void modeHandler(const std_msgs::UInt8::ConstPtr &message);
void targetHandler(const shared_messages::TagsImage::ConstPtr &tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr &message); // 
void odometryHandler(const nav_msgs::Odometry::ConstPtr &message);
void mobilityStateMachine(const ros::TimerEvent &);
void publishStatusTimerEventHandler(const ros::TimerEvent &event);
void killSwitchTimerEventHandler(const ros::TimerEvent &event);
void messageHandler(const std_msgs::String::ConstPtr &message);
void target_collectedHandler(const std_msgs::String::ConstPtr &message);

int main(int argc, char **argv)
{
	gethostname(host, sizeof(host));
	string hostName(host);
	rng = new random_numbers::RandomNumberGenerator(); // instantiate random number generator
	if (argc >= 2)
	{
		rover_name = argv[1];
		cout << "Welcome to the world of tomorrow " << rover_name << "!  Mobility module started." << endl;
	} else
	{
		rover_name = hostName;
		cout << "No Name Selected. Default is: " << rover_name << endl;
	}
	// NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
	ros::init(argc, argv, (rover_name + "_MOBILITY"), ros::init_options::NoSigintHandler);
	ros::NodeHandle mNH;

	signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

	joySubscriber = mNH.subscribe((rover_name + "/joystick"), 10, joyCmdHandler);
	modeSubscriber = mNH.subscribe((rover_name + "/mode"), 1, modeHandler);
	targetSubscriber = mNH.subscribe((rover_name + "/targets"), 10, targetHandler);
	obstacleSubscriber = mNH.subscribe((rover_name + "/obstacle"), 10, obstacleHandler);
	odometrySubscriber = mNH.subscribe((rover_name + "/odom/ekf"), 10, odometryHandler);
	messageSubscriber = mNH.subscribe(("messages"), 10, messageHandler);
	target_collectedSubscriber = mNH.subscribe(("targetsCollected"), 100, target_collectedHandler);

	status_publisher = mNH.advertise<std_msgs::String>((rover_name + "/status"), 1, true);
	velocityPublish = mNH.advertise<geometry_msgs::Twist>((rover_name + "/velocity"), 10);
	stateMachinePublish = mNH.advertise<std_msgs::String>((rover_name + "/state_machine"), 1, true);
	messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
	target_collected_publisher = mNH.advertise<std_msgs::String>(("targetsCollected"), 100, true);
	angular_publisher = mNH.advertise<std_msgs::String>((rover_name + "/angular"),1,true);
	publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
	killSwitchTimer = mNH.createTimer(ros::Duration(kill_switch_timeout), killSwitchTimerEventHandler);
	stateMachineTimer = mNH.createTimer(ros::Duration(mobility_loop_time_step), mobilityStateMachine);
	debug_publisher = mNH.advertise<std_msgs::String>("debug", 100, true);
	messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10 , true);

	ros::spin();
	return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent &)
{
	std_msgs::String state_machine_msg;
	std_msgs::String pose_msg;
	std_msgs::String debug_msg;
	std::stringstream converter1;
	std_msgs::String debug_msg1;
	std::stringstream converter2;
	ros::Rate rate(10);


	float angular_velocity = 0;
	float linear_velocity = 0;
	if ((simulation_mode == 2 || simulation_mode == 3)) // Robot is in automode
	{
		if (transitions_to_auto == 0)
		{
			// This is the first time we have clicked the Autonomous Button. Log the time and increment the counter.
			transitions_to_auto++;
			time_stamp_transition_to_auto = ros::Time::now().toSec();
			outputFile.open("/home/administrator/log_project/test1.txt");
			outputFile <<"initial auto start time =" << time_stamp_transition_to_auto<<"\n";
			outputFile.close(); 
		}
		switch (state_machine_state)
		{
			case STATE_MACHINE_TRANSLATE:
				{
					if (pro_start_target_search==true){
						//pro_goal_theta_temp= rng->uniformReal(0, 2 * M_PI);
						pro_goal_location.x=rng->uniformReal(-6,6);
						pro_goal_location.y=rng->uniformReal(-6,6);
						//pro_dist_total=DISTANCE_FOR_TARVEL;
					}
						pro_goal_theta_temp= atan2(pro_goal_location.y-current_location.y,pro_goal_location.x-current_location.x);
					if(fabs(current_location.theta-pro_goal_theta_temp)<=0.08){
						if (hypot(pro_goal_location.x-current_location.x, pro_goal_location.y-current_location.y)>=.5){
							angular_velocity = 0.0;
							linear_velocity = (hypot(pro_goal_location.x-current_location.x, pro_goal_location.y-current_location.y))*0.15;
							if (linear_velocity>=LINEAR_VELOCITY){
								linear_velocity=LINEAR_VELOCITY;
							}else if (linear_velocity<=-LINEAR_VELOCITY){
								linear_velocity = -LINEAR_VELOCITY;
							}
							setVelocity(linear_velocity, angular_velocity);
						}
							else{
								state_machine_state=STATE_MACHINE_TEMP;

							}
						}
					else{
						pro_start_target_search=false;
    						
						angular_velocity = angles::shortest_angular_distance(current_location.theta, pro_goal_theta_temp)*0.15;
						linear_velocity = 0.0;
						setVelocity(linear_velocity, angular_velocity);
					}
					temp_time_stamp=ros::Time::now().toSec();
					//if ((pro_count_total_collected==255)||(temp_time_stamp>=600000))
					if ((pro_count_total_collected==255)||((temp_time_stamp-time_stamp_transition_to_auto)>3500))
					{
						state_machine_state=STATE_STOP_TIME_UP;
					}
					break;
				}

			case STATE_MACHINE_TEMP:
				{
					if (pro_targetdetected==false){
						pro_start_target_search=true;
						state_machine_state=STATE_MACHINE_TRANSLATE;
					}else {
						pro_start_home_search=true;
						state_machine_state=STATE_MACHINE_GO_HOME;
					}

					break;
				}
			case STATE_MACHINE_GO_HOME:
				{
					
					if (pro_start_home_search==true){
						//pro_goal_theta_temp= rng->uniformReal(0, 2 * M_PI);
						pro_goal_location.x=rng->uniformReal(-6,6);
						pro_goal_location.y=rng->uniformReal(-6,6);
						//pro_dist_total=DISTANCE_FOR_TARVEL;
					}
						pro_goal_theta_temp= atan2(pro_goal_location.y-current_location.y,pro_goal_location.x-current_location.x);
					if(fabs(current_location.theta-pro_goal_theta_temp)<=0.08){
						if (hypot(pro_goal_location.x-current_location.x, pro_goal_location.y-current_location.y)>=.5){
							angular_velocity = 0.0;
							linear_velocity = (hypot(pro_goal_location.x-current_location.x, pro_goal_location.y-current_location.y))*0.15;
							if (linear_velocity>=LINEAR_VELOCITY){
								linear_velocity=LINEAR_VELOCITY;
							}else if (linear_velocity<=-LINEAR_VELOCITY){
								linear_velocity = -LINEAR_VELOCITY;
							}
							setVelocity(linear_velocity, angular_velocity);
						}
							else{
								state_machine_state=STATE_MACHINE_TEMP;

							}
						}
					else{
						pro_start_home_search=false;
    						
						angular_velocity = angles::shortest_angular_distance(current_location.theta, pro_goal_theta_temp)*0.15;
						linear_velocity = 0.0;
						setVelocity(linear_velocity, angular_velocity);
					}
					temp_time_stamp=ros::Time::now().toSec();
					//if ((pro_count_total_collected==255)||(temp_time_stamp>=600000))
					if ((pro_count_total_collected==255)||((temp_time_stamp-time_stamp_transition_to_auto)>3500))
					{
						state_machine_state=STATE_STOP_TIME_UP;
					}
					break;
				}
			case STATE_STOP_TIME_UP:
			{
				std_msgs::String debug_msg;
				std::stringstream converter1;
				double stop_time=0.0; 
				converter1 << "inside stop simulation "<<  "\n";
				debug_msg.data=converter1.str();
				debug_publisher.publish(debug_msg);
				stop_time=ros::Time::now().toSec(); 
				outputFile.open("/home/administrator/log_project/test1.txt", std::ofstream::out | std::ofstream::app);
				outputFile <<"end simulation time=" <<(stop_time-time_stamp_transition_to_auto)<<"\n" ;
				outputFile <<"total targets collected =" <<pro_count_total_collected<<"\n" ;
				outputFile.close(); 
				setVelocity(0.0, 0.0);
				pro_start_home_search=false;
				pro_start_target_search=false;
				simulation_mode=1;	
				break;
			}
			default:
				{
					state_machine_msg.data = "DEFAULT CASE: SOMETHING WRONG!!!!";
					break;
				}
		}

	}
	else
	{ // mode is NOT auto

		// publish current state for the operator to seerotational_controller
		std::stringstream converter;
		converter <<"CURRENT MODE: " << simulation_mode;

		state_machine_msg.data = "WAITING, " + converter.str();
	}
	stateMachinePublish.publish(state_machine_msg);
	//pose_msg.data = "I am ";
}

void setVelocity(double linearVel, double angularVel)
{
	geometry_msgs::Twist velocity;
	// Stopping and starting the timer causes it to start counting from 0 again.
	// As long as this is called before the kill switch timer reaches kill_switch_timeout seconds
	// the rover's kill switch wont be called.
	killSwitchTimer.stop();
	killSwitchTimer.start();

	velocity.linear.x = linearVel * 1.5;
	velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
	velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/
//void targetHandler(const shared_messages::TagsImage::ConstPtr &message) {
// Only used if we want to take action after seeing an April Tag.
//} // code added at end from prof jason.

void modeHandler(const std_msgs::UInt8::ConstPtr &message)
{
	simulation_mode = message->data;
	setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr &message)
{
	if ( message->data > 0 )
	{
		if (message->data == 1)
		{
			// obstacle on right side
		}
		else
		{
			//obstacle in front or on left side
		}
	}
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &message)
{
	//Get (x,y) location directly from pose
	current_location.x = message->pose.pose.position.x;
	current_location.y = message->pose.pose.position.y;

	//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y,
			message->pose.pose.orientation.z, message->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	current_location.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message)
{
	if (simulation_mode == 0 || simulation_mode == 1)
	{
		setVelocity(message->linear.x, message->angular.z);
	}
}

void publishStatusTimerEventHandler(const ros::TimerEvent &)
{
	if (!is_published_name)
	{
		std_msgs::String name_msg;
		name_msg.data = "I ";
		name_msg.data = name_msg.data + rover_name;
		messagePublish.publish(name_msg);
		is_published_name = true;
	}

	std_msgs::String msg;
	msg.data = "online";
	status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent &t)
{
	// No movement commands for killSwitchTime seconds so stop the rover
	setVelocity(0.0, 0.0);
	double current_time = ros::Time::now().toSec();
	ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.",
			current_time);
}

void sigintEventHandler(int sig)
{
	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

void messageHandler(const std_msgs::String::ConstPtr& message)
{
}
void targetHandler(const shared_messages::TagsImage::ConstPtr &message) {
	// If in manual mode do not try to automatically pick up the target
	if (simulation_mode == 1 || simulation_mode == 0) return;

	if (pro_targetdetected == false) {
		process_resources(message);
	}
	//else if (state_machine_state==STATE_MACHINE_GO_HOME && !center_seen) //not sure why center_seen is used
	else if (state_machine_state==STATE_MACHINE_GO_HOME)
	{
		process_home_tags(message);
	}
}

void process_resources(const shared_messages::TagsImage::ConstPtr &message) {
	std_msgs::Int16 targetDetected;
	for (int i = 0; i < message->tags.data.size(); i++) {
		//check if target has not yet been collected
		if (message->tags.data[i] != 256 && !pro_targets_collected[message->tags.data[i]]) {
			pro_target_collected = true;
			//copy target ID to class variable
			//targetDetected.data = message->tags.data[i];
			//publish detected target
			//target_collected_publisher.publish(targetDetected);
			//publish to scoring code
			//targetPickUpPublish.publish(message->image);
			pro_count_target_collected++;
			pro_targetdetected = true;
			std_msgs::String pro_claimed_id;
			std::stringstream converter;
			converter << " " <<pro_count_target_collected<< " " <<message->tags.data[i];
			pro_claimed_id.data = rover_name;
			pro_claimed_id.data= pro_claimed_id.data + converter.str();
			target_collected_publisher.publish(pro_claimed_id);	
			state_machine_state=STATE_MACHINE_GO_HOME;	
			pro_start_home_search=true;
			//claimed_id.data = message->tags.data[i];
			//pickup_publisher.publish(claimed_id);
			return;
		}
	}
}

void process_home_tags(const shared_messages::TagsImage::ConstPtr &message) {
	std_msgs::String debug_msg;
	for (int i = 0; i < message->tags.data.size(); i++) {
		int tag_id = message->tags.data[i];
		if (tag_id == 256) {
			/*center_seen = true;
			  if (targetDetected.data != -1) {
			//publish to scoring code
			targetDropOffPublish.publish(message->image);
			targetDetected.data = -1;
			return;
			}*/
			if (pro_targetdetected == true){	
				pro_targetdetected =false;
				state_machine_state=STATE_MACHINE_TRANSLATE;
				pro_start_target_search=true;
				std::stringstream converter1;
				converter1 << "the home delivered target ID " << message->tags.data[i] <<  "\n";
				debug_msg.data=converter1.str();
				debug_publisher.publish(debug_msg);
			}
		}
	}
	//center_seen = false;
}
void target_collectedHandler(const std_msgs::String::ConstPtr &message){

	size_t pos = 0;
	int i=0,j=0;
	std::string delimiter = " ";
	std::string token;
	std::stringstream converter;
	std_msgs::String debug_msg;
	converter << message->data;
	std::string message1 = converter.str();
	pos = message1.find(delimiter);
	token = message1.substr(0, pos);
	for(i=0;i<3;i++){
		if (token.compare(rovers1[i].name)==0)
			break;
	}
	message1.erase(0, pos + delimiter.length());
	if(pos !=std::string::npos){
		pos = message1.find(delimiter);
		token = message1.substr(0, pos);
		rovers1[i].pro_target_collected_data=atoi(token.c_str());
	}
	message1.erase(0, pos + delimiter.length());
	if(pos !=std::string::npos){
		pos = message1.find(delimiter);
		token = message1.substr(0, pos);
		rovers1[i].pro_target_id=atoi(token.c_str());
	}
	pro_count_total_collected = rovers1[0].pro_target_collected_data +rovers1[1].pro_target_collected_data + rovers1[2].pro_target_collected_data+rovers1[3].pro_target_collected_data +rovers1[4].pro_target_collected_data + rovers1[5].pro_target_collected_data;
	pro_targets_collected[rovers1[i].pro_target_id]= true;
	pro_time_stamp_target_collected = ros::Time::now().toSec();
			outputFile.open("/home/administrator/log_project/test1.txt", std::ofstream::out | std::ofstream::app);
			outputFile <<"target collected at time=" <<(pro_time_stamp_target_collected-time_stamp_transition_to_auto)<"\n" ;
			outputFile.close(); 

	std::stringstream converter1;
	converter1 << "rover name "<<rovers1[i].name <<" the number of collected tragets " << pro_count_total_collected << " ID of target collected now" << rovers1[i].pro_target_id << "\n";
	debug_msg.data=converter1.str();
	debug_publisher.publish(debug_msg);
}
