#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
using namespace std;


class posePublisher{
	ros::NodeHandle nh;
	geometry_msgs::PoseStamped msg1,msg2,msg3,msg4,msg5,msg6,msg7;
	ros::Publisher pub1, pub2, pub3, pub4, pub5, pub6, pub7;
public:
	posePublisher(){
		pub1 = nh.advertise<geometry_msgs::PoseStamped>("/pose1", 100);
		pub2 = nh.advertise<geometry_msgs::PoseStamped>("/pose2", 100);
		pub3 = nh.advertise<geometry_msgs::PoseStamped>("/pose3", 100);
		pub4 = nh.advertise<geometry_msgs::PoseStamped>("/pose4", 100);
		pub5 = nh.advertise<geometry_msgs::PoseStamped>("/pose5", 100);
		pub6 = nh.advertise<geometry_msgs::PoseStamped>("/pose6", 100);
		pub7 = nh.advertise<geometry_msgs::PoseStamped>("/pose7", 100);
	}
	void assignVals(geometry_msgs::PoseStamped &msg, double count){
		msg.pose.orientation.x = count;
    	msg.pose.orientation.y = count;
    	msg.pose.orientation.z = count;
    	msg.pose.orientation.w = count;

    	msg.pose.position.x = count;
    	msg.pose.position.y = count;
    	msg.pose.position.z = count;
    	count++;
	}
	void PublishVals(){
		assignVals(msg1,1);
		assignVals(msg2,2);
		assignVals(msg3,3);
		assignVals(msg4,4);
		assignVals(msg5,5);
		assignVals(msg6,6);
		assignVals(msg7,7);
		//cout<<msg1.pose.orientation.x;
		pub1.publish(msg1);
		pub2.publish(msg2);
		pub3.publish(msg3);
		pub4.publish(msg4);
		pub5.publish(msg5);
		pub6.publish(msg6);
		pub7.publish(msg7);
	}
};



int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "fakePub");
  
  posePublisher obj;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
  	obj.PublishVals();
  	//ros::spin();
    ros::spinOnce(); 
 	loop_rate.sleep();
 }
  
  //ros::spin();
  return 0;
}