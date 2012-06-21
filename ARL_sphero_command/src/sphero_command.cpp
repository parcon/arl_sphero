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

#include <std_msgs/Float32MultiArray.h>

char dest[18] = "00:06:66:44:66:04"; //*** SPHERO ID ***

float new_msg[4];
bool newmsg = false;
bool new_vel_msg = true;
uint8_t PKT[11];
uint8_t PKT_vel[11];
uint8_t PKT_stop[11];
uint8_t PKT_red[11];
uint8_t PKT_blue[11];
uint8_t PKT_yellow[11];
uint8_t PKT_boost[10];
uint8_t PKT_head[9];
double the_time;

//Packet pieces
uint8_t SOP1 = 0xFF;
uint8_t SOP2_responsewanted = 0xFF;
uint8_t SOP2_noresponsewanted = 0xFE;
uint8_t DID = 0xFF; //Device ID:The virtual device this packet is intended for
uint8_t CID = 0xFF; //Command ID: The command code
uint8_t SEQ = 0xFF; //This client field is echoed in the response for all synchronous commands (and ignored by Sphero when SOP2 = FEh)
uint8_t DLEN = 0xFF;//The number of bytes following through the end of the packet
uint8_t DATA = 0xFF; //Optional data to accompany the Command
uint8_t CHK = 0xFF; //The modulo 256 sum of all the bytes from the DID through the end of the data payload, bit inverted (1's complement)

/*
//call back for new data comming in
void msg_cb(const std_msgs::Float32MultiArray& msg)
{
	new_msg[0]=msg.data[0];
	newmsg =true;
}
*/

void create_velocity_message(uint8_t spd,uint16_t hed, uint8_t* PKT)
{
	uint8_t DID = 0x02;
	uint16_t CID =0x30;
	uint8_t SEQ= 0x01; 
	uint8_t DLEN = 0x05;
	uint8_t SPEED = spd; //8 bits or 1 byte 0-256 //0xFF is max speed (all ones)
	//uint16_t Heading =0x0000; // 0 degrees
	/* The client convention for heading follows the 360 degrees on a circle, relative to the ball: 0 is straight
	ahead, 90 is to the right, 180 is back and 270 is to the left. The valid range is 0..359
	*/
	uint8_t HEAD_1= uint8_t(hed >> 8); //<most significant bits>
	uint8_t HEAD_2= uint8_t(hed); //<least significant bits>
	uint8_t STATE =  1;  //<bool> set to 0
	uint8_t sum= (DID+CID+SEQ+DLEN+SPEED+HEAD_1+HEAD_2+STATE)%256;
	CHK= sum^0xFF;
	uint8_t data_to_sphero[11] = {SOP1,SOP2_noresponsewanted,DID,CID,SEQ,DLEN,SPEED,HEAD_1,HEAD_2,STATE,CHK};

	for (int i=0; i< 11; i++)
	{
		PKT[i] = data_to_sphero[i];
	}
	ROS_INFO("Velocity Message Created");

} //create vel msg

void create_color_message(uint8_t red_,uint8_t green_, uint8_t blue_, uint8_t flag_,uint8_t* PKT)
{
	uint8_t DID = 0x02;
	uint16_t CID =0x20;
	uint8_t SEQ= 0x01; 
	uint8_t DLEN = 0x05;
	uint8_t RED = red_; //8 bits or 1 byte 0-256 
	uint8_t BLUE= blue_; 
	uint8_t GREEN= green_; 
	uint8_t FLAG =  flag_;  //decides if this color will be the given color persistant across sphero reboots
	uint8_t sum= (DID+CID+SEQ+DLEN+RED+BLUE+GREEN+FLAG)%256;
	CHK= sum^0xFF;
	uint8_t data_to_sphero[11] = {SOP1,SOP2_noresponsewanted,DID,CID,SEQ,DLEN,RED,BLUE,GREEN,FLAG,CHK};

	for (int i=0; i< 11; i++)
	{
		PKT[i] = data_to_sphero[i];
	}
	ROS_INFO("Color Message Created");

} //create color msg

void create_boost_message(uint16_t hed,uint16_t time_, uint8_t* PKT)
{
	uint8_t DID = 0x02;
	uint16_t CID =0x31;
	uint8_t SEQ= 0x01; 
	uint8_t DLEN = 0x04;
	uint8_t TIME_BOOST = time_;
	uint8_t HEAD_1= uint8_t(hed >> 8); //<most significant bits>
	uint8_t HEAD_2= uint8_t(hed); //<least significant bits>
	uint8_t sum= (DID+CID+SEQ+DLEN+TIME_BOOST+HEAD_1+HEAD_2)%256;
	CHK= sum^0xFF;
	uint8_t data_to_sphero[10] = {SOP1,SOP2_noresponsewanted,DID,CID,SEQ,DLEN,TIME_BOOST,HEAD_1,HEAD_2,CHK};

	for (int i=0; i< 10; i++)
	{
		PKT[i] = data_to_sphero[i];
	}
	ROS_INFO("Boost Message Created");

} //create boost msg

void create_heading_message(uint16_t hed,uint8_t* PKT)
{
	uint8_t DID = 0x02;
	uint16_t CID =0x01;
	uint8_t SEQ= 0x01; 
	uint8_t DLEN = 0x03;
	uint8_t HEAD_1= uint8_t(hed >> 8); //<most significant bits>
	uint8_t HEAD_2= uint8_t(hed); //<least significant bits>
	uint8_t sum= (DID+CID+SEQ+DLEN+HEAD_1+HEAD_2)%256;
	CHK= sum^0xFF;
	uint8_t data_to_sphero[9] = {SOP1,SOP2_noresponsewanted,DID,CID,SEQ,DLEN,HEAD_1,HEAD_2,CHK};

	for (int i=0; i< 9; i++)
	{
		PKT[i] = data_to_sphero[i];
	}
	ROS_INFO("Heading Message Created");

} //create head msg


int main(int argc, char **argv)
{

	ros::init(argc, argv, "Sphero_Command");
	ros::NodeHandle n;
	ros::Rate r(5); //update @ 5 hz

	//ros::Publisher cmd_pub = n.advertise<std_msgs::UInt16MultiArray>("Quad_Cmd", 10);
	//ros::Subscriber err_sub = n.subscribe("sphero_cmds", 100, msg_cb);
	struct sockaddr_rc addr = { 0 };
	int s, status;
    
    ROS_INFO("Starting Sphero Command...");

    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);     // allocate a socket

   
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba(dest, &addr.rc_bdaddr );  // set the connection parameters (who to connect to)
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));     // connect to server
    


	
	create_color_message(255,0,0,1,PKT_red);
	create_color_message(0,255,0,1,PKT_blue);
	create_color_message(250,250,210,1,PKT_yellow);
	
	create_velocity_message(255,0,PKT_vel);
	create_velocity_message(0,0,PKT_stop);
	create_boost_message(255,5,PKT_boost);
	create_heading_message(180,PKT_head);
	the_time=(double)ros::Time::now().toSec();

//while (ros::ok())
//{

    // send a message
    if( status == 0 ) 
    {
//    while ((double)ros::Time::now().toSec() < the_time +3)
 //   {
    	status = write(s, PKT_red, sizeof(PKT_red)); 
    	sleep(1);
    	status = write(s, PKT_green, sizeof(PKT_green)); 
    	sleep(1);
    	status = write(s, PKT_yellow, sizeof(PKT_yellow)); 
		sleep(1);
		ROS_INFO("Color Message Sent");
	//}
    	status = write(s, PKT_head, sizeof(PKT_head)); 	
		ROS_INFO("Heading Message Sent");
    	sleep(2);
    	
    	status = write(s, PKT_vel, sizeof(PKT_vel)); 	
		ROS_INFO("Velocity Message Sent");
		sleep(3);
		
		status = write(s, PKT_stop, sizeof(PKT_stop)); 	
		ROS_INFO("Stop Message Sent");
    
	}
    if( status < 0 ) 
	{
		ROS_ERROR("Device Busy/Broken/Oops");
	} 
   
	close(s);
	r.sleep();
//}

}
