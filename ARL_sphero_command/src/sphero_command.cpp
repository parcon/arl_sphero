/*
 *  Copyright (c) 2012, Parker Conroy
 *	ARLab @ University of Utah
 *  All rights reserved.
 *
 *
 *	The purpose of this software is to take a command message such as a /joy or /twist and drive a sphero with said message.
 *
 *
 *
 *	This software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <unistd.h>
#include <string.h> //maybe
#include <strings.h> //maybe
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include "ros/time.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "sphero_messages.h"
#define PI 3.14159265

char dest[18] = "00:06:66:44:66:04"; //*** SPHERO ID ***
float new_msg[4];
bool newmsg = false;
bool new_vel_msg = true;
float velocity, heading;
sensor_msgs::Joy old_msg;
//geometry_msgs::Twist old_msg;
uint8_t PKT[11];
uint8_t PKT_vel[11];
uint8_t PKT_stop[11];
uint8_t PKT_red[11];
uint8_t PKT_blue[11];
uint8_t PKT_yellow[11];
uint8_t PKT_boost[10];
uint8_t PKT_head[9];
uint8_t PKT_backled[8];
double the_time;


//call backs
void msg_cb(const sensor_msgs::Joy& new_msg)
{

	//for twist messages
		//velocity= sqrt(pow(new_msg.linear.x,2) +pow(new_msg.linear.y,2));
		//heading= atan2(new_msg.linear.y,new_msg.linear.x)*180/PI;
		//create_velocity_message(velocity,heading,PKT_vel);

	//for joy messages
		velocity= (uint8_t)(sqrt(pow(new_msg.axes[0],2) +pow(new_msg.axes[1],2))*(255/0.43));
		heading= (uint16_t)(atan2(new_msg.axes[1],new_msg.axes[0])*180/PI -98); //98 calibarates so zero is forward
		while (heading > 360)
		{ heading -= 360;
		}
		if (heading < 0)
		{
		heading += 360;
		}
		create_velocity_message(velocity,heading,PKT_vel);
		ROS_INFO("Velocity: %f Heading: %f", velocity, heading);
		

	old_msg=new_msg;
	the_time=(double)ros::Time::now().toSec();
}

//end call backs

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Sphero_Command");
	ros::NodeHandle n;
	ros::Rate r(1); //update @ 1hz
	//ros::Publisher cmd_pub = n.advertise<std_msgs::UInt16MultiArray>("Quad_Cmd", 10);
	ros::Subscriber velocity_sub = n.subscribe("/joy", 1, msg_cb);
	
	struct sockaddr_rc addr = { 0 };
	int s, status;
    ROS_INFO("Starting Sphero Command...");
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);     // allocate a socket

    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba(dest, &addr.rc_bdaddr );  // set the connection parameters (who to connect to)
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));     // connect to server

	//create the messages to be used
	create_color_message(255,0,0,1,PKT_red);
	create_color_message(0,255,0,1,PKT_blue);
	create_color_message(250,250,210,1,PKT_yellow);
	create_backled_message(255,PKT_backled);
	
	//create_velocity_message(255,0,PKT_vel);
	create_velocity_message(0,0,PKT_stop);
	//create_boost_message(255,5,PKT_boost);
	//create_heading_message(180,PKT_head);
	//the_time=(double)ros::Time::now().toSec();
		
    if( status == 0 ) //if socket is open
    {
		for (int i=0; i >3; i++)
		{
			status = write(s, PKT_red, sizeof(PKT_red)); 
			sleep(.5);
			status = write(s, PKT_yellow, sizeof(PKT_yellow)); 
			sleep(.5);
		}

		status = write(s, PKT_backled, sizeof(PKT_backled)); 
		sleep(1);
		ROS_INFO("Lighting Messages Sent");
	
		while (ros::ok()) 
			{
				ros::spinOnce();
				status = write(s, PKT_vel, sizeof(PKT_vel)); 	
				ROS_INFO("Velocity Message Sent, Status: %f",status);
		

				if ( status < 0 )
				{
					ROS_ERROR("Device Busy/Broken/Oops");
				}
				//status = write(s, PKT_stop, sizeof(PKT_stop)); 	
				//ROS_INFO("Stop Message Sent");
		  
				//close(s); //close the socket when done
				r.sleep();
			}//while ros ok
	}// if socket is ok
}//main
