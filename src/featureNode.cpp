#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

const int dimension=7;
double normVal;

using namespace std;

class FormFeatureVector{
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub1,sub2,sub3,sub4,sub5,sub6,sub7;
	std_msgs::Float64MultiArray featureVec;
	double x_vals[dimension], y_vals[dimension], z_vals[dimension];
	//double dist[dimension*dimension];
public:
	void callBack1(const geometry_msgs::PoseStamped msg)
	{
  		x_vals[0]= msg.pose.position.x;
  		y_vals[0]= msg.pose.position.y;
  		z_vals[0]= msg.pose.position.z;

	}
	void callBack2(const geometry_msgs::PoseStamped msg)
	{
  		x_vals[1]= msg.pose.position.x;
  		y_vals[1]= msg.pose.position.y;
  		z_vals[1]= msg.pose.position.z;
  	}
  	
  	void callBack3(const geometry_msgs::PoseStamped msg)
	{
  		x_vals[2]= msg.pose.position.x;
  		y_vals[2]= msg.pose.position.y;
  		z_vals[2]= msg.pose.position.z;
  	}
  	void callBack4(const geometry_msgs::PoseStamped msg)
	{
  		x_vals[3]= msg.pose.position.x;
  		y_vals[3]= msg.pose.position.y;
  		z_vals[3]= msg.pose.position.z;
  	}
  	void callBack5(const geometry_msgs::PoseStamped msg)
	{
  		x_vals[4]= msg.pose.position.x;
  		y_vals[4]= msg.pose.position.y;
  		z_vals[4]= msg.pose.position.z;
  	}
  	void callBack6(const geometry_msgs::PoseStamped msg)
	{
  		x_vals[5]= msg.pose.position.x;
  		y_vals[5]= msg.pose.position.y;
  		z_vals[5]= msg.pose.position.z;
  	}
  	void callBack7(const geometry_msgs::PoseStamped msg)
	{
  		x_vals[6]= msg.pose.position.x;
  		y_vals[6]= msg.pose.position.y;
  		z_vals[6]= msg.pose.position.z;
  		featureVec.data.clear();
  		calcDist();
  		pub.publish(featureVec);
  	}
  	void calcDist(){
  		double dist;
  		for(int i=0;i<dimension;i++){
  			for(int j=0;j<dimension;j++){
  				dist=sqrt(pow((x_vals[i]-x_vals[j]),2)+pow((y_vals[i]-y_vals[j]),2)+pow((z_vals[i]-z_vals[j]),2));
  				dist=dist/normVal;
  				featureVec.data.push_back(dist);
  			}
  		}
  	}

	FormFeatureVector(){
		sub1 = nh.subscribe("/pose1", 100, &FormFeatureVector::callBack1, this);
		sub2 = nh.subscribe("/pose2", 100, &FormFeatureVector::callBack2, this);
		sub3 = nh.subscribe("/pose3", 100, &FormFeatureVector::callBack3, this);
		sub4 = nh.subscribe("/pose4", 100, &FormFeatureVector::callBack4, this);
		sub5 = nh.subscribe("/pose5", 100, &FormFeatureVector::callBack5, this);
		sub6 = nh.subscribe("/pose6", 100, &FormFeatureVector::callBack6, this);
		sub7 = nh.subscribe("/pose7", 100, &FormFeatureVector::callBack7, this);
		pub = nh.advertise<std_msgs::Float64MultiArray>("featureVector", 100);
	}
	
};



int main(int argc, char **argv)
{
    
 /* 
  while (ros::ok())
  {
  	
  	
    ros::spin();
 }
 */
  cout<<"enter hand to hand distance in T pose"<<endl;
  cin>>normVal;
  ros::init(argc, argv, "featureNode");
  FormFeatureVector obj;
  ros::spin();
  return 0;
}
