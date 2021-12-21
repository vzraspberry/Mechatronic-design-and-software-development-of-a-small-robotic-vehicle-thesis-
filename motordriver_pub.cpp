#include <ros/ros.h>
#include <stdlib.h>
#include <serial/serial.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

serial::Serial ser;

extern float vel = 0;
extern float avel = 0;

int main(int argc, char **argv){

//****************************Init*****************************

	ros::init(argc, argv, "motordriver_pub");

	ros::NodeHandle n;

	ros::Publisher motordriver_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
	//ros::Publisher motordriver_pub = n.advertise<std_msgs::Bool>("motor_power", 1000);

	ros::Rate loop_rate(30);

	float velocity = 0;
	float avelocity = 0;

//***********************USB connection ***********************

	try{
		ser.setPort("/dev/ttyACM0");
		ser.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(2000);
		ser.setTimeout(to);
		ser.open();
	}
	
	catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

//*************************Publishing***************************
	
	while (ros::ok()){
		
		//libusb_handle_events_timeout(...);
		
		//std_msgs::Bool;
		//msg.data = true;
		
		velocity = vel;
		avelocity = avel;
		
		geometry_msgs::Twist msg;
		msg.linear.x = velocity;
		msg.angular.z = avelocity;
		
		motordriver_pub.publish(msg);
	}
	return 1;
}