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


using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator *rng;

typedef struct
{
	float x;
	float y;
	float theta;
	float dist_rover1;
	float dist_rover2;
	float dist_rover3;
	float dist_rover4;
	float dist_rover5;
	string name;
}rover_pose_ori;
//Change for Homework4 Question 2
rover_pose_ori global_rover_value = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"global"};
rover_pose_ori local_rover_value = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"local"};
rover_pose_ori local_rover_cohesion = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"coshesion"};
rover_pose_ori local_rover_seperation = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"seperation"};
rover_pose_ori rovers[6]={{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"ajax"},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"achilles"},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"aeneas"},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"diomedes"},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"hector"},{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"paris"}};
float theta_from_global_pos=0.0;
float theta_from_average_pos=0.0;

//Change for Homework4 Question 2

string rover_name;
char host[128];
bool is_published_name = false;


int simulation_mode = 0;
float mobility_loop_time_step = 0.1;
float status_publish_interval = 5;
float kill_switch_timeout = 10;

pose current_location;
//Change for Homework4 Question 2
pose average_location;
pose cohesion_location;
pose combined_location;

//Change for Homework4 Question 2
int transitions_to_auto = 0;
double time_stamp_transition_to_auto = 0.0;

// state machine states
#define STATE_MACHINE_TRANSLATE 0
int state_machine_state = STATE_MACHINE_TRANSLATE;

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
ros::Subscriber messageSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

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
void poseHandler(const std_msgs::String::ConstPtr &message);
void global_average_headingHandler(const std_msgs::String::ConstPtr &message);
void local_average_headingHandler(const std_msgs::String::ConstPtr &message);

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
	poseSubscriber = mNH.subscribe(("pose"), 1000, poseHandler);
	global_average_headingSubscriber = mNH.subscribe(("global_average_heading"), 1000, global_average_headingHandler);
	local_average_headingSubscriber = mNH.subscribe(("local_average_heading"), 1000, local_average_headingHandler);

	status_publisher = mNH.advertise<std_msgs::String>((rover_name + "/status"), 1, true);
	velocityPublish = mNH.advertise<geometry_msgs::Twist>((rover_name + "/velocity"), 10);
	stateMachinePublish = mNH.advertise<std_msgs::String>((rover_name + "/state_machine"), 1, true);
	messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
	target_collected_publisher = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
	angular_publisher = mNH.advertise<std_msgs::String>((rover_name + "/angular"),1,true);
	publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
	killSwitchTimer = mNH.createTimer(ros::Duration(kill_switch_timeout), killSwitchTimerEventHandler);
	stateMachineTimer = mNH.createTimer(ros::Duration(mobility_loop_time_step), mobilityStateMachine);
	debug_publisher = mNH.advertise<std_msgs::String>("/debug", 1, true);
	messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10 , true);
	posePublish = mNH.advertise<std_msgs::String>(("pose"), 1000 , true);
	global_average_headingPublish = mNH.advertise<std_msgs::String>(("global_average_heading"), 1000 , true);
	local_average_headingPublish = mNH.advertise<std_msgs::String>(("local_average_heading"), 1000 , true);

	ros::spin();
	return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent &)
{
	std_msgs::String state_machine_msg;
	std_msgs::String pose_msg;
	float kp=.1;
	float angular_velocity = 0;
	float linear_velocity = 0;

	if ((simulation_mode == 2 || simulation_mode == 3)) // Robot is in automode
	{
		if (transitions_to_auto == 0)
		{
			// This is the first time we have clicked the Autonomous Button. Log the time and increment the counter.
			transitions_to_auto++;
			time_stamp_transition_to_auto = ros::Time::now().toSec();
		}
		switch (state_machine_state)
		{
			case STATE_MACHINE_TRANSLATE:
				{
					/*average_location.x=current_location.x-local_rover_value.x;
					average_location.y=current_location.y-local_rover_value.y;
					theta_from_average_pos=atan2(average_location.y,average_location.x);
					cohesion_location.x=current_location.x-local_rover_cohesion.x;
					cohesion_location.y=current_location.y-local_rover_cohesion.y;
					theta_from_global_pos=atan2(cohesion_location.y,cohesion_location.x);*/
//Change for Homework4 Question 2
					combined_location.x= (local_rover_value.x+local_rover_cohesion.x+local_rover_seperation.x)/3;
					combined_location.y=(local_rover_value.y+local_rover_cohesion.y+local_rover_seperation.y)/3;
					theta_from_global_pos=atan2(combined_location.y,combined_location.x);
					angular_velocity = kp*(theta_from_global_pos-current_location.theta);
//Change for Homework4 Question 2
					//average_location.x=current_location.x-global_rover_value.x;
					//average_location.y=current_location.y-global_rover_value.y;
					//theta_from_average_pos=atan2(average_location.y,average_location.x);
					//angular_velocity = kp*(local_rover_value.theta- current_location.theta);
					//angular_velocity = kp*(global_rover_value.theta-current_location.theta);
					//	angular_velocity =kp*(theta_from_average_pos-current_location.theta);
					state_machine_msg.data = "TRANSLATING";//, " + converter.str();
					//angular_velocity = 0.2;
					linear_velocity = 0.05;
					setVelocity(linear_velocity, angular_velocity);
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
	pose_msg.data = rover_name;
	std::stringstream converter;
	converter << " " << current_location.x << " " << current_location.y << " " << current_location.theta;
	pose_msg.data = pose_msg.data +converter.str();
	posePublish.publish(pose_msg);
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
void targetHandler(const shared_messages::TagsImage::ConstPtr &message) {
	// Only used if we want to take action after seeing an April Tag.
}

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
void poseHandler(const std_msgs::String::ConstPtr& message)
{
	size_t pos = 0;
	std_msgs::String global_average_heading_msg;
	std_msgs::String local_average_heading_msg;
	int i=0,dist_between,count=0;
	std::stringstream converter;
	std::stringstream converter1;
	std::stringstream converter2;
	converter << message->data;
	std::string message1 = converter.str();
	std::string delimiter = " ";
	std::string token;
	pos = message1.find(delimiter);
	token = message1.substr(0, pos);
	for(i=0;i<6;i++){
		if (token.compare(rovers[i].name)==0)
			break;
	}
	//rovers[0].name=token;
	message1.erase(0, pos + delimiter.length());
	if(pos !=std::string::npos){
		pos = message1.find(delimiter);
		token = message1.substr(0, pos);
		//rovers[i].x=std::stof(token);
		rovers[i].x=atof(token.c_str());
	}
	message1.erase(0, pos + delimiter.length());
	if(pos !=std::string::npos){
		pos = message1.find(delimiter);
		token = message1.substr(0, pos);
		//rovers[i].y=std::stof(token);
		rovers[i].y=atof(token.c_str());
	}
	message1.erase(0, pos + delimiter.length());
	if(pos !=std::string::npos){
		pos = message1.find(delimiter);
		token = message1.substr(0, pos);
		//rovers[i].theta=std::stof(token);
		rovers[i].theta=atof(token.c_str());
	}
	global_rover_value.name="global";
	global_rover_value.x=(rovers[1].x+rovers[2].x+rovers[0].x+rovers[3].x+rovers[4].x+rovers[5].x)/6;
	global_rover_value.y=(rovers[1].y+rovers[2].y+rovers[0].y+rovers[3].y+rovers[4].y+rovers[5].y)/6;
	global_rover_value.theta=atan2(sin(rovers[0].theta)+sin(rovers[1].theta)+sin(rovers[2].theta)+sin(rovers[3].theta)+sin(rovers[4].theta)+sin(rovers[5].theta),cos(rovers[0].theta)+cos(rovers[1].theta)+cos(rovers[2].theta)+cos(rovers[3].theta)+cos(rovers[4].theta)+cos(rovers[5].theta));
	converter1 << global_rover_value.name << " " << global_rover_value.x << " " << global_rover_value.y << " " << global_rover_value.theta;
	global_average_heading_msg.data = converter1.str();	
	//global_average_heading_msg.data = message1;	
	global_average_headingPublish.publish(global_average_heading_msg);
	local_rover_value.x=0;
	local_rover_value.y=0;
	local_rover_cohesion.x=0;
	local_rover_cohesion.y=0;
	local_rover_seperation.x=0;
	local_rover_seperation.y=0;
//Change for Homework4 Question 2
	switch(i){
		case 0:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_value.x=rovers[1].x;
				local_rover_value.y=rovers[1].y;
				local_rover_value.theta= atan2(sin(rovers[1].theta),cos(rovers[1].theta));
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_value.x=local_rover_value.x+rovers[2].x;
				local_rover_value.y=local_rover_value.y+rovers[2].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[2].theta),cos(local_rover_value.theta)+cos(rovers[2].theta));
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_value.x=local_rover_value.x+rovers[3].x;
				local_rover_value.y=local_rover_value.y+rovers[3].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[3].theta),cos(local_rover_value.theta)+cos(rovers[3].theta));
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_value.x=local_rover_value.x+rovers[4].x;
				local_rover_value.y=local_rover_value.y+rovers[4].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[4].theta),cos(local_rover_value.theta)+cos(rovers[4].theta));
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_value.x=local_rover_value.x+rovers[5].x;
				local_rover_value.y=local_rover_value.y+rovers[5].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[5].theta),cos(local_rover_value.theta)+cos(rovers[5].theta));
				count++;
			}
			count++;
			local_rover_value.x=(local_rover_value.x+current_location.x)/count;
			local_rover_value.y=(local_rover_value.y+current_location.y)/count;
			local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(current_location.theta),cos(local_rover_value.theta)+cos(current_location.theta));
			local_rover_value.name="local";
			converter2 << local_rover_value.name << " " << local_rover_value.x << " " << local_rover_value.y << " " << local_rover_value.theta;
			local_average_heading_msg.data = converter2.str();	
			local_average_headingPublish.publish(local_average_heading_msg);
			count=0;
			break;
		case 1:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_value.x=rovers[0].x;
				local_rover_value.y=rovers[0].y;
				local_rover_value.theta= atan2(sin(rovers[0].theta),cos(rovers[0].theta));
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_value.x=local_rover_value.x+rovers[2].x;
				local_rover_value.y=local_rover_value.y+rovers[2].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[2].theta),cos(local_rover_value.theta)+cos(rovers[2].theta));
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_value.x=local_rover_value.x+rovers[3].x;
				local_rover_value.y=local_rover_value.y+rovers[3].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[3].theta),cos(local_rover_value.theta)+cos(rovers[3].theta));
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_value.x=local_rover_value.x+rovers[4].x;
				local_rover_value.y=local_rover_value.y+rovers[4].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[4].theta),cos(local_rover_value.theta)+cos(rovers[4].theta));
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_value.x=local_rover_value.x+rovers[5].x;
				local_rover_value.y=local_rover_value.y+rovers[5].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[5].theta),cos(local_rover_value.theta)+cos(rovers[5].theta));
				count++;
			}
			count++;
			local_rover_value.x=(local_rover_value.x+current_location.x)/count;
			local_rover_value.y=(local_rover_value.y+current_location.y)/count;
			local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(current_location.theta),cos(local_rover_value.theta)+cos(current_location.theta));
			local_rover_value.name="local";
			converter2 << local_rover_value.name << " " << local_rover_value.x << " " << local_rover_value.y << " " << local_rover_value.theta;
			local_average_heading_msg.data = converter2.str();	
			local_average_headingPublish.publish(local_average_heading_msg);
			count=0;
			break;
		case 2:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_value.x=rovers[0].x;
				local_rover_value.y=rovers[0].y;
				local_rover_value.theta= atan2(sin(rovers[0].theta),cos(rovers[0].theta));
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_value.x=local_rover_value.x+rovers[1].x;
				local_rover_value.y=local_rover_value.y+rovers[1].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[1].theta),cos(local_rover_value.theta)+cos(rovers[1].theta));
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_value.x=local_rover_value.x+rovers[3].x;
				local_rover_value.y=local_rover_value.y+rovers[3].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[3].theta),cos(local_rover_value.theta)+cos(rovers[3].theta));
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_value.x=local_rover_value.x+rovers[4].x;
				local_rover_value.y=local_rover_value.y+rovers[4].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[4].theta),cos(local_rover_value.theta)+cos(rovers[4].theta));
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_value.x=local_rover_value.x+rovers[5].x;
				local_rover_value.y=local_rover_value.y+rovers[5].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[5].theta),cos(local_rover_value.theta)+cos(rovers[5].theta));
				count++;
			}
			count++;
			local_rover_value.x=(local_rover_value.x+current_location.x)/count;
			local_rover_value.y=(local_rover_value.y+current_location.y)/count;
			local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(current_location.theta),cos(local_rover_value.theta)+cos(current_location.theta));
			local_rover_value.name="local";
			converter2 << local_rover_value.name << " " << local_rover_value.x << " " << local_rover_value.y << " " << local_rover_value.theta;
			local_average_heading_msg.data = converter2.str();	
			local_average_headingPublish.publish(local_average_heading_msg);
			count=0;
			break;
		case 3:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_value.x=rovers[0].x;
				local_rover_value.y=rovers[0].y;
				local_rover_value.theta= atan2(sin(rovers[0].theta),cos(rovers[0].theta));
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_value.x=local_rover_value.x+rovers[1].x;
				local_rover_value.y=local_rover_value.y+rovers[1].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[1].theta),cos(local_rover_value.theta)+cos(rovers[1].theta));
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_value.x=local_rover_value.x+rovers[2].x;
				local_rover_value.y=local_rover_value.y+rovers[2].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[2].theta),cos(local_rover_value.theta)+cos(rovers[2].theta));
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_value.x=local_rover_value.x+rovers[4].x;
				local_rover_value.y=local_rover_value.y+rovers[4].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[4].theta),cos(local_rover_value.theta)+cos(rovers[4].theta));
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_value.x=local_rover_value.x+rovers[5].x;
				local_rover_value.y=local_rover_value.y+rovers[5].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[5].theta),cos(local_rover_value.theta)+cos(rovers[5].theta));
				count++;
			}
			count++;
			local_rover_value.x=(local_rover_value.x+current_location.x)/count;
			local_rover_value.y=(local_rover_value.y+current_location.y)/count;
			local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(current_location.theta),cos(local_rover_value.theta)+cos(current_location.theta));
			local_rover_value.name="local";
			converter2 << local_rover_value.name << " " << local_rover_value.x << " " << local_rover_value.y << " " << local_rover_value.theta;
			local_average_heading_msg.data = converter2.str();	
			local_average_headingPublish.publish(local_average_heading_msg);
			count=0;
			break;
		case 4:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_value.x=rovers[0].x;
				local_rover_value.y=rovers[0].y;
				local_rover_value.theta= atan2(sin(rovers[0].theta),cos(rovers[0].theta));
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_value.x=local_rover_value.x+rovers[1].x;
				local_rover_value.y=local_rover_value.y+rovers[1].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[1].theta),cos(local_rover_value.theta)+cos(rovers[1].theta));
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_value.x=local_rover_value.x+rovers[2].x;
				local_rover_value.y=local_rover_value.y+rovers[2].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[2].theta),cos(local_rover_value.theta)+cos(rovers[2].theta));
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_value.x=local_rover_value.x+rovers[3].x;
				local_rover_value.y=local_rover_value.y+rovers[3].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[3].theta),cos(local_rover_value.theta)+cos(rovers[3].theta));
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_value.x=local_rover_value.x+rovers[5].x;
				local_rover_value.y=local_rover_value.y+rovers[5].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[5].theta),cos(local_rover_value.theta)+cos(rovers[5].theta));
				count++;
			}
			count++;
			local_rover_value.x=(local_rover_value.x+current_location.x)/count;
			local_rover_value.y=(local_rover_value.y+current_location.y)/count;
			local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(current_location.theta),cos(local_rover_value.theta)+cos(current_location.theta));
			local_rover_value.name="local";
			converter2 << local_rover_value.name << " " << local_rover_value.x << " " << local_rover_value.y << " " << local_rover_value.theta;
			local_average_heading_msg.data = converter2.str();	
			local_average_headingPublish.publish(local_average_heading_msg);
			count=0;
			break;
		case 5:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_value.x=rovers[0].x;
				local_rover_value.y=rovers[0].y;
				local_rover_value.theta= atan2(sin(rovers[0].theta),cos(rovers[0].theta));
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_value.x=local_rover_value.x+rovers[1].x;
				local_rover_value.y=local_rover_value.y+rovers[1].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[1].theta),cos(local_rover_value.theta)+cos(rovers[1].theta));
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_value.x=local_rover_value.x+rovers[2].x;
				local_rover_value.y=local_rover_value.y+rovers[2].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[2].theta),cos(local_rover_value.theta)+cos(rovers[2].theta));
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_value.x=local_rover_value.x+rovers[3].x;
				local_rover_value.y=local_rover_value.y+rovers[3].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[3].theta),cos(local_rover_value.theta)+cos(rovers[3].theta));
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_value.x=local_rover_value.x+rovers[4].x;
				local_rover_value.y=local_rover_value.y+rovers[4].y;
				local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(rovers[4].theta),cos(local_rover_value.theta)+cos(rovers[4].theta));
				count++;
			}
			count++;
			local_rover_value.x=(local_rover_value.x+current_location.x)/count;
			local_rover_value.y=(local_rover_value.y+current_location.y)/count;
			local_rover_value.theta= atan2(sin(local_rover_value.theta)+sin(current_location.theta),cos(local_rover_value.theta)+cos(current_location.theta));
			local_rover_value.name="local";
			converter2 << local_rover_value.name << " " << local_rover_value.x << " " << local_rover_value.y << " " << local_rover_value.theta;
			local_average_heading_msg.data = converter2.str();	
			local_average_headingPublish.publish(local_average_heading_msg);
			count=0;
			break;
		default:
			cout << "no valid case";
			break;
	}
	switch(i){
		case 0:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=0.3){
				local_rover_seperation.x=current_location.x-rovers[1].x;
				local_rover_seperation.y=current_location.y-rovers[1].y;
				count++;
			}
			if(rovers[i].dist_rover2<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[2].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[2].y);
				count++;
			}
			if(rovers[i].dist_rover3<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[3].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[3].y);
				count++;
			}
			if(rovers[i].dist_rover4<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[4].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[4].y);
				count++;
			}
			if(rovers[i].dist_rover5<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[5].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[5].y);
				count++;
			}
			count++;
			local_rover_seperation.x=(local_rover_seperation.x+current_location.x)/count;
			local_rover_seperation.y=(local_rover_seperation.y+current_location.y)/count;
			local_rover_seperation.name="seperation";
			count=0;
			break;
		case 1:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=0.3){
				local_rover_seperation.x=current_location.x-rovers[0].x;
				local_rover_seperation.y=current_location.y-rovers[0].y;
				count++;
			}
			if(rovers[i].dist_rover2<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[2].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[2].y);
				count++;
			}
			if(rovers[i].dist_rover3<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[3].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[3].y);
				count++;
			}
			if(rovers[i].dist_rover4<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[4].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[4].y);
				count++;
			}
			if(rovers[i].dist_rover5<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[5].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[5].y);
				count++;
			}
			count++;
			local_rover_seperation.x=(local_rover_seperation.x+current_location.x)/count;
			local_rover_seperation.y=(local_rover_seperation.y+current_location.y)/count;
			local_rover_seperation.name="seperation";
			count=0;
			break;
		case 2:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=0.3){
				local_rover_seperation.x=current_location.x-rovers[0].x;
				local_rover_seperation.y=current_location.y-rovers[0].y;
				count++;
			}
			if(rovers[i].dist_rover2<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[1].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[1].y);
				count++;
			}
			if(rovers[i].dist_rover3<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[3].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[3].y);
				count++;
			}
			if(rovers[i].dist_rover4<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[4].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[4].y);
				count++;
			}
			if(rovers[i].dist_rover5<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[5].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[5].y);
				count++;
			}
			count++;
			local_rover_seperation.x=(local_rover_seperation.x+current_location.x)/count;
			local_rover_seperation.y=(local_rover_seperation.y+current_location.y)/count;
			local_rover_seperation.name="seperation";
			count=0;
			break;
		case 3:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=0.3){
				local_rover_seperation.x=current_location.x-rovers[0].x;
				local_rover_seperation.y=current_location.y-rovers[0].y;
				count++;
			}
			if(rovers[i].dist_rover2<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[1].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[1].y);
				count++;
			}
			if(rovers[i].dist_rover3<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[2].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[2].y);
				count++;
			}
			if(rovers[i].dist_rover4<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[4].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[4].y);
				count++;
			}
			if(rovers[i].dist_rover5<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[5].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[5].y);
				count++;
			}
			count++;
			local_rover_seperation.x=(local_rover_seperation.x+current_location.x)/count;
			local_rover_seperation.y=(local_rover_seperation.y+current_location.y)/count;
			local_rover_seperation.name="seperation";
			count=0;
			break;
		case 4:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=0.3){
				local_rover_seperation.x=current_location.x-rovers[0].x;
				local_rover_seperation.y=current_location.y-rovers[0].y;
				count++;
			}
			if(rovers[i].dist_rover2<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[1].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[1].y);
				count++;
			}
			if(rovers[i].dist_rover3<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[2].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[2].y);
				count++;
			}
			if(rovers[i].dist_rover4<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[3].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[3].y);
				count++;
			}
			if(rovers[i].dist_rover5<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[5].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[5].y);
				count++;
			}
			count++;
			local_rover_seperation.x=(local_rover_seperation.x+current_location.x)/count;
			local_rover_seperation.y=(local_rover_seperation.y+current_location.y)/count;
			local_rover_seperation.name="seperation";
			count=0;
			break;
		case 5:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			if(rovers[i].dist_rover1<=0.3){
				local_rover_seperation.x=current_location.x-rovers[0].x;
				local_rover_seperation.y=current_location.y-rovers[0].y;
				count++;
			}
			if(rovers[i].dist_rover2<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[1].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[1].y);
				count++;
			}
			if(rovers[i].dist_rover3<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[2].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[2].y);
				count++;
			}
			if(rovers[i].dist_rover4<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[3].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[3].y);
				count++;
			}
			if(rovers[i].dist_rover5<=0.3){
				local_rover_seperation.x=local_rover_seperation.x+(current_location.x-rovers[4].x);
				local_rover_seperation.y=local_rover_seperation.y+(current_location.y-rovers[4].y);
				count++;
			}
			count++;
			local_rover_seperation.x=(local_rover_seperation.x+current_location.x)/count;
			local_rover_seperation.y=(local_rover_seperation.y+current_location.y)/count;
			local_rover_seperation.name="seperation";
			count=0;
			break;
		default:
			cout << "no valid case";
			break;
	}
	switch(i){
		case 0:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_cohesion.x=rovers[1].x-current_location.x;
				local_rover_cohesion.y=rovers[1].y-current_location.y;
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[2].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[2].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[3].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[3].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[4].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[4].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[5].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[5].y-current_location.y);
				count++;
			}
			count++;
			local_rover_cohesion.x=(local_rover_cohesion.x+current_location.x)/count;
			local_rover_cohesion.y=(local_rover_cohesion.y+current_location.y)/count;
			local_rover_cohesion.name="cohesion";
			count=0;
			break;
		case 1:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_cohesion.x=rovers[0].x-current_location.x;
				local_rover_cohesion.y=rovers[0].y-current_location.y;
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[2].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[2].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[3].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[3].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[4].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[4].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[5].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(current_location.y-rovers[5].y-current_location.y);
				count++;
			}
			count++;
			local_rover_cohesion.x=(local_rover_cohesion.x+current_location.x)/count;
			local_rover_cohesion.y=(local_rover_cohesion.y+current_location.y)/count;
			local_rover_cohesion.name="cohesion";
			count=0;
			break;
		case 2:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_cohesion.x=rovers[0].x-current_location.x;
				local_rover_cohesion.y=rovers[0].y-current_location.y;
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[1].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[1].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[3].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[3].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[4].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[4].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[5].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[5].y-current_location.y);
				count++;
			}
			count++;
			local_rover_cohesion.x=(local_rover_cohesion.x+current_location.x)/count;
			local_rover_cohesion.y=(local_rover_cohesion.y+current_location.y)/count;
			local_rover_cohesion.name="cohesion";
			count=0;
			break;
		case 3:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_cohesion.x=rovers[0].x-current_location.x;
				local_rover_cohesion.y=rovers[0].y-current_location.y;
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[1].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[1].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[2].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[2].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[4].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[4].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[5].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[5].y-current_location.y);
				count++;
			}
			count++;
			local_rover_cohesion.x=(local_rover_cohesion.x+current_location.x)/count;
			local_rover_cohesion.y=(local_rover_cohesion.y+current_location.y)/count;
			local_rover_cohesion.name="cohesion";
			count=0;
			break;
		case 4:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[5].x),2))+(pow((current_location.y-rovers[5].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_cohesion.x=rovers[0].x-current_location.x;
				local_rover_cohesion.y=rovers[0].y-current_location.y;
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[1].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[1].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[2].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[2].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[3].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[3].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[5].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[5].y-current_location.y);
				count++;
			}
			count++;
			local_rover_cohesion.x=(local_rover_cohesion.x+current_location.x)/count;
			local_rover_cohesion.y=(local_rover_cohesion.y+current_location.y)/count;
			local_rover_cohesion.name="cohesion";
			count=0;
			break;
		case 5:
			rovers[i].dist_rover1=sqrt((pow((current_location.x-rovers[0].x),2))+(pow((current_location.y-rovers[0].y),2)));
			rovers[i].dist_rover2=sqrt((pow((current_location.x-rovers[1].x),2))+(pow((current_location.y-rovers[1].y),2)));
			rovers[i].dist_rover3=sqrt((pow((current_location.x-rovers[2].x),2))+(pow((current_location.y-rovers[2].y),2)));
			rovers[i].dist_rover4=sqrt((pow((current_location.x-rovers[3].x),2))+(pow((current_location.y-rovers[3].y),2)));
			rovers[i].dist_rover5=sqrt((pow((current_location.x-rovers[4].x),2))+(pow((current_location.y-rovers[4].y),2)));
			if(rovers[i].dist_rover1<=2){
				local_rover_cohesion.x=rovers[0].x-current_location.x;
				local_rover_cohesion.y=rovers[0].y-current_location.y;
				count++;
			}
			if(rovers[i].dist_rover2<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[1].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[1].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover3<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[2].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[2].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover4<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[3].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[3].y-current_location.y);
				count++;
			}
			if(rovers[i].dist_rover5<=2){
				local_rover_cohesion.x=local_rover_cohesion.x+(rovers[4].x-current_location.x);
				local_rover_cohesion.y=local_rover_cohesion.y+(rovers[4].y-current_location.y);
				count++;
			}
			count++;
			local_rover_cohesion.x=(local_rover_cohesion.x+current_location.x)/count;
			local_rover_cohesion.y=(local_rover_cohesion.y+current_location.y)/count;
			local_rover_cohesion.name="cohesion";
			count=0;
			break;
		default:
			cout << "no valid case";
			break;
	}

}
void global_average_headingHandler(const std_msgs::String::ConstPtr& message)
{
}
void local_average_headingHandler(const std_msgs::String::ConstPtr& message)
{
}
//Change for Homework4 Question 2
