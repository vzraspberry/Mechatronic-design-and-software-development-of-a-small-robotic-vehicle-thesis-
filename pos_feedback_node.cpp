#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>
using namespace std;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "position_feedback");
	
	void publishPoseStamped(void){
		
	ros::Publisher position_feedback = nh.advertise<geometry_msgs::PoseStamped>("position", 1000);
		
	ros::Rate loop_rate(30);
		
	while (ros::ok()){
			
		geometry_msgs::PoseStamped poseStamped;
		poseStamped.header.frame_id="/position";
		poseStamped.header.stamp = ros::Time::now();
						
		goal.pose.position.x = odom.pose.pose.position.x;
		goal.pose.position.y = odom.pose.pose.position.y;
		goal.pose.orientation = odom.pose.pose.orientation;
			
		poseStampedPub.publish(poseStamped);
	}
	return 1;
}