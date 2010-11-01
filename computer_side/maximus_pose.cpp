/*! \class TeleopMaximus
 *  \brief Class to communicate between the robot and ROS.
 *  \author Joffrey Kriegel
 *  \version 1.0
 *  \date    2010
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <joy/Joy.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                               // for in-/output
#include <string.h>                              // strcat
#include <fcntl.h>                               // for 'O_RDONLY' deklaration
#include <termios.h>                             // for serial

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>

#include <vector>

#define BAUDRATE B115200

class TeleopMaximus
{
	public:
		TeleopMaximus();
		void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped *pose);
		signed int read_serial_port(char first_input);
		void publish_all(void);
		void get_value_and_do_computation(void);

		// Serial
		int ser_fd;
		struct termios oldtio, newtio;

		// Joystick suscriber
		ros::Subscriber joy_sub_;
		// Set the position of the robot
		ros::Publisher pose_in_map_pub;
		// Set the path of the robot => Doesn't work ...
		ros::Publisher path_in_map_pub;
		// Set a shape for the robot
		ros::Publisher marker_pub;
		// Set a Laser scan sensor for the robot
		ros::Publisher laser_sensor_pub;


	private:
		void joyCallback(const joy::Joy::ConstPtr& joy);
		ros::NodeHandle nh;

		int i;

		int linear_port, angular_port;
		double l_scale_, a_scale_;
		double linear_value, angular_value, prev_linear_value, prev_angular_value;
		float heading;

		tf::TransformBroadcaster br;
		tf::Transform transform;
		sensor_msgs::LaserScan my_laser_scan;
		visualization_msgs::Marker marker;
		geometry_msgs::PoseStamped my_maximus_pose;
		nav_msgs::Path my_maximus_path;


};

TeleopMaximus::TeleopMaximus():
	linear_port(1),
	angular_port(2),
	a_scale_(12000),
	l_scale_(12000)
{

	nh.param("axis_linear", linear_port, linear_port);
	nh.param("axis_angular", angular_port, angular_port);
	nh.param("scale_angular", a_scale_, a_scale_);
	nh.param("scale_linear", l_scale_, l_scale_);

	linear_value = 0;
	angular_value = 0;
	prev_linear_value = 0;
	prev_angular_value = 0;

	heading = 0;

	// Joystick suscriber
	joy_sub_ = nh.subscribe<joy::Joy>("joy", 10, &TeleopMaximus::joyCallback, this);
	// Set the position of the robot
	pose_in_map_pub  = nh.advertise<geometry_msgs::PoseStamped>("maximus_pose", 50);
	// Set the path of the robot 
	path_in_map_pub  = nh.advertise<nav_msgs::Path>("maximus_path", 50);
	// Set a shape for the robot
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	// Set a Laser scan sensor for the robot
	laser_sensor_pub = nh.advertise<sensor_msgs::LaserScan>("laser_scan_sensor", 50);

	ser_fd = open("/dev/rfcomm1", O_RDWR | O_NOCTTY | O_NONBLOCK);
	if( ser_fd == -1)
	{
		printf( " Serial Not Open \n" );
	}
	else
	{
		printf( " Serial Open \n" );
		tcgetattr(ser_fd, &oldtio);                             // Backup old port settings
		memset(&newtio, 0, sizeof(newtio));

		newtio.c_iflag = IGNBRK | IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_cflag = BAUDRATE | CREAD | CS8 | CLOCAL;
		newtio.c_lflag = 0;

		//newtio.c_cc[VTIME]     = 0;  /* inter-character timer unused */
		//newtio.c_cc[VMIN]     = 1;   /* blocking read until 6 chars received */

		tcflush(ser_fd, TCIFLUSH);
		tcsetattr(ser_fd, TCSANOW, &newtio);

		memset(&newtio, 0, sizeof(newtio));
		tcgetattr(ser_fd, &newtio);

		fcntl(ser_fd, F_SETFL, FNDELAY);
	}

	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/maximus_robot_tf"));

	my_laser_scan.header.frame_id = "/maximus_robot_tf";
	my_laser_scan.header.stamp = ros::Time::now();

	my_laser_scan.angle_min = -0.52;
	my_laser_scan.angle_max = 0.52;
	my_laser_scan.angle_increment = 0.52;
	// time between measurements [seconds]
	my_laser_scan.time_increment = 0;
	// time between scans [seconds]
	my_laser_scan.scan_time = 0.1;
	my_laser_scan.range_min = 0.10;
	my_laser_scan.range_max = 0.80;

	// range data [m] (Note: values < range_min or > range_max should be discarded)
	my_laser_scan.ranges = std::vector<float>();
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.15);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.35);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);


	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/maximus_robot_tf";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::CUBE;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.32;
	marker.scale.y = 0.31;
	marker.scale.z = 0.4;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;


	my_maximus_pose.header.frame_id = "/map";
	my_maximus_pose.header.stamp = ros::Time::now();

	my_maximus_pose.pose.position.x = 0;
	my_maximus_pose.pose.position.y = 0;
	my_maximus_pose.pose.position.z = 0;
	my_maximus_pose.pose.orientation.x = 0;
	my_maximus_pose.pose.orientation.y = 0;
	my_maximus_pose.pose.orientation.z = 0;
	my_maximus_pose.pose.orientation.w = 0;


	my_maximus_path.header.frame_id = "/map";
	my_maximus_path.header.stamp = ros::Time::now();

	my_maximus_path.poses = std::vector<geometry_msgs::PoseStamped>();

	if(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::size() > (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::max_size()-2)) {
		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::pop_back();
	}
	my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::push_back(my_maximus_pose);



	TeleopMaximus::rotate(((0)*3.1415/180), 0, 0, &my_maximus_pose);

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 0.11;
	marker.pose.position.y = 0;
	marker.pose.position.z = marker.scale.z/2;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 0;


	i = 0;

}


void TeleopMaximus::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped *pose) {
	// Assuming the angles are in radians.
	double c1 = cos(heading/2);
	double s1 = sin(heading/2);
	double c2 = cos(attitude/2);

	double s2 = sin(attitude/2);
	double c3 = cos(bank/2);
	double s3 = sin(bank/2);
	double c1c2 = c1*c2;
	double s1s2 = s1*s2;

	pose->pose.orientation.w = c1c2*c3 - s1s2*s3;
	pose->pose.orientation.x = c1c2*s3 + s1s2*c3;
	pose->pose.orientation.y = s1*c2*c3 + c1*s2*s3;
	pose->pose.orientation.z = c1*s2*c3 - s1*c2*s3;
}

signed int TeleopMaximus::read_serial_port(char first_input) {

	char my_input[50];
	signed int temp_input;
	char read_flag = 0;

	temp_input = 0;


	read_flag = read(ser_fd, my_input, 1);
	while( (my_input[0] != first_input) ) {
		read_flag = read(ser_fd, my_input, 1);
	}



	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	if(my_input[0] == '-') { // Negative number
		read_flag = read(ser_fd, my_input, 1);
		while((read_flag == 0) || (read_flag == -1)) {
			read_flag = read(ser_fd, my_input, 1);
		}
		temp_input = (temp_input * 10) - (my_input[0] - 48);
		read_flag = read(ser_fd, my_input, 1);
		while((read_flag == 0) || (read_flag == -1)) {
			read_flag = read(ser_fd, my_input, 1);
		}
		temp_input = (temp_input * 10) - (my_input[0] - 48);
		read_flag = read(ser_fd, my_input, 1);
		while((read_flag == 0) || (read_flag == -1)) {
			read_flag = read(ser_fd, my_input, 1);
		}
		temp_input = (temp_input * 10) - (my_input[0] - 48);
		read_flag = read(ser_fd, my_input, 1);
		while((read_flag == 0) || (read_flag == -1)) {
			read_flag = read(ser_fd, my_input, 1);
		}
		temp_input = (temp_input * 10) - (my_input[0] - 48);
		read_flag = read(ser_fd, my_input, 1);
		while((read_flag == 0) || (read_flag == -1)) {
			read_flag = read(ser_fd, my_input, 1);
		}
		temp_input = (temp_input * 10) - (my_input[0] - 48);
		read_flag = read(ser_fd, my_input, 1);
		while((read_flag == 0) || (read_flag == -1)) {
			read_flag = read(ser_fd, my_input, 1);
		}
		temp_input = (temp_input * 10) - (my_input[0] - 48);
	}
	else { // Positive number
		temp_input = (temp_input * 10) + (my_input[0] - 48);
		read_flag = read(ser_fd, my_input, 1);
		while((read_flag == 0) || (read_flag == -1)) {
			read_flag = read(ser_fd, my_input, 1);
		}
		temp_input = (temp_input * 10) + (my_input[0] - 48);
		read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
			read_flag = read(ser_fd, my_input, 1);
		}
		temp_input = (temp_input * 10) + (my_input[0] - 48);
		read_flag = read(ser_fd, my_input, 1);
		while((read_flag == 0) || (read_flag == -1)) {
			read_flag = read(ser_fd, my_input, 1);
		}
		temp_input = (temp_input * 10) + (my_input[0] - 48);
		read_flag = read(ser_fd, my_input, 1);
		while((read_flag == 0) || (read_flag == -1)) {
			read_flag = read(ser_fd, my_input, 1);
		}
		temp_input = (temp_input * 10) + (my_input[0] - 48);
		read_flag = read(ser_fd, my_input, 1);
		while((read_flag == 0) || (read_flag == -1)) {
			read_flag = read(ser_fd, my_input, 1);
		}
		temp_input = (temp_input * 10) + (my_input[0] - 48);
	}
	return temp_input;
}

void TeleopMaximus::joyCallback(const joy::Joy::ConstPtr& joy)
{
	angular_value = a_scale_*joy->axes[angular_port];
	linear_value = l_scale_*joy->axes[linear_port];
}

void TeleopMaximus::get_value_and_do_computation(void) {
	float rotation = 0.0;

	if(ser_fd != -1) {
		my_maximus_pose.pose.position.y = -((float)TeleopMaximus::read_serial_port('x')) / 10000.0;
		my_maximus_pose.pose.position.x = -((float)TeleopMaximus::read_serial_port('y')) / 10000.0;
		rotation =  -TeleopMaximus::read_serial_port('t');
		TeleopMaximus::rotate(0, (-( (float)rotation) / 10000.0 ), 0, &my_maximus_pose);

		// reset laser scan
		//                      my_laser_scan.ranges.std::vector<float>::clear();
		// set the new values
		//my_laser_scan.ranges.std::vector<float>::push_back(((float)read_serial_port('l')) / 1000.0);
		//my_laser_scan.ranges.std::vector<float>::push_back(((float)read_serial_port('m')) / 1000.0);
		//my_laser_scan.ranges.std::vector<float>::push_back(((float)read_serial_port('r')) / 1000.0);

	}
	else {
		//TeleopMaximus::rotate(0, ((i)*3.1415/180), 0, &my_maximus_pose);
		//rotation = -((i)*3.1415/180) *10000;
		//my_maximus_pose.pose.position.y = ((float)i)/40;
		heading -= (angular_value / 120000 );
		rotation = heading *10000;
		TeleopMaximus::rotate(0, -(heading), 0, &my_maximus_pose);
		
		my_maximus_pose.pose.position.x += (linear_value * cos(-heading) / 100000);
		my_maximus_pose.pose.position.y += (linear_value * sin(-heading) / 100000);

	}

	// Actualize Robot's position
	my_maximus_pose.header.stamp = ros::Time::now();

	// Actualize Robot's path
	my_maximus_path.header.stamp = ros::Time::now();
	if(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::size() > (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::max_size()-2)) {
		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::pop_back();
	}
	my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::push_back(my_maximus_pose);

	marker.lifetime = ros::Duration();

	// Publish the marker
	//marker.header.stamp = ros::Time::now();
	//marker_pub.publish(marker);

	transform.setOrigin( tf::Vector3(my_maximus_pose.pose.position.x, my_maximus_pose.pose.position.y, 0.0) );
	transform.setRotation( tf::Quaternion((-( (float)rotation) / 10000.0 ), 0, 0) );
	br.sendTransform(tf::StampedTransform(transform, my_maximus_pose.header.stamp, "/map", "/maximus_robot_tf"));

	marker.header.stamp = ros::Time::now();

	my_laser_scan.header.stamp = ros::Time::now();

	i = i + 5;
	if( i > 180)
		i = -180;
	ROS_INFO("X=%f /Y=%f /T=%f /L=%f", my_maximus_pose.pose.position.x, my_maximus_pose.pose.position.y, my_maximus_pose.pose.orientation.z*180/3.1415, linear_value);

}


void TeleopMaximus::publish_all(void) {
	char Serout[260]={0};
	// Publish Pose of the robot
	TeleopMaximus::pose_in_map_pub.publish(my_maximus_pose);
	// Publish Path of the robot
	TeleopMaximus::path_in_map_pub.publish(my_maximus_path);
	// Publish the marker
	//marker.header.stamp = ros::Time::now();
	//marker_pub.publish(marker);

	// Publish the shape of the robot
	TeleopMaximus::marker_pub.publish(marker);

	// Publish the laser scan values
	TeleopMaximus::laser_sensor_pub.publish(my_laser_scan);
	// Send acceleration values to the robot
	if(ser_fd != -1) {
	/*if ( ser_fd ) {*/
		if(prev_linear_value != linear_value) {
			char byte1, byte2, byte3, byte4, byte5;
			byte1 = (char)(((long)abs(linear_value)) % 10)+48;
			byte2 = (char)((((long)abs(linear_value)) /10) % 10)+48;
			byte3 = (char)((((long)abs(linear_value)) /100) % 10)+48;
			byte4 = (char)((((long)abs(linear_value)) /1000) % 10)+48;
			byte5 = (char)((((long)abs(linear_value)) /10000) % 10)+48;
			
			if(linear_value >= 0)
				sprintf(Serout, "l%c", byte5);
			else
				sprintf(Serout, "L%c", byte5);
			sprintf(Serout, "%s%c", Serout, byte4);
			sprintf(Serout, "%s%c", Serout, byte3);
			sprintf(Serout, "%s%c", Serout, byte2);
			sprintf(Serout, "%s%c", Serout, byte1);

			//sprintf(Serout, "l05000");
                	write(ser_fd, &Serout, 6);
			prev_linear_value = linear_value;
			ROS_INFO("linear %s", Serout);
		}
		if(prev_angular_value != angular_value) {
         		char byte1, byte2, byte3, byte4, byte5;
			byte1 = (char)(((long)abs(angular_value)) % 10)+48;
                        byte2 = (char)((((long)abs(angular_value)) /10) % 10)+48;
                        byte3 = (char)((((long)abs(angular_value)) /100) % 10)+48;
                        byte4 = (char)((((long)abs(angular_value)) /1000) % 10)+48;
                        byte5 = (char)((((long)abs(angular_value)) /10000) % 10)+48;

                        if(angular_value >= 0)
                                sprintf(Serout, "a%c", byte5);
                        else
                                sprintf(Serout, "A%c", byte5);
                        sprintf(Serout, "%s%c", Serout, byte4);
                        sprintf(Serout, "%s%c", Serout, byte3);
                        sprintf(Serout, "%s%c", Serout, byte2);
                        sprintf(Serout, "%s%c", Serout, byte1);

                        //sprintf(Serout, "l05000");
                        write(ser_fd, &Serout, 6);
       
			prev_angular_value = angular_value;
			ROS_INFO("angular %lu", (long)prev_angular_value);
                }

	}
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

	// write to serial if connected
	//if ( ser_fd ) {
	//	sprintf(Serout, "S"); // Start the communication
	//	write(ser_fd, &Serout, sizeof(Serout));
	//}

	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "maximus_talker");
	TeleopMaximus maximus_talker;
	// Refresh rate
	ros::Rate loop_rate(35); // 35 with bluetooth
	float rotation = 0.0;
	while (ros::ok())
	{
		// Get the values and do the computation
		maximus_talker.get_value_and_do_computation();
		// Publish all the values and messages
		maximus_talker.publish_all();
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}




