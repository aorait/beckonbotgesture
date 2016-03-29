#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <list>
#include "GRT.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#define dimension 4
#define buffer_size 260
#define likelyhood_threshold 0.3

using namespace GRT;


class MyFrame
{
public:
	double vecData[dimension];

	MyFrame(double array[dimension])
	{
		for (int i=0; i<dimension; i++)  //the "normalization" is here
		{
			vecData[i] = (double)((int)(array[i]*500));
		}
	}
};

class Livetest
{
	ros::NodeHandle nh;
	ros::Publisher pub;  //publish to topic "finalLabel"
	ros::Subscriber sub1;  //listen to topic "featureVector"
	TimeSeriesClassificationData testingData;
	MatrixDouble testingSample;
	int receivedFrameNum;  //let's keep buffer size, sample at granularity of 5, start at receiving buffer size
	std::list<MyFrame> buffer;
	
	GestureRecognitionPipeline pipeline;
public:

	void callBack(const std_msgs::Float64MultiArray::ConstPtr& msg)
	{
		receivedFrameNum ++;
		double array[dimension];
		int i = 0;
		for(std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
		{
			array[i] = *it;
			i++;
		}
		printf("Alive? 1\n");
		VectorDouble sample( dimension );
		for (int j=0; j<dimension; j++)
		{
			sample[j] = array[j];
		}
		printf("Alive? 2\n");
		pipeline.predict(sample);
		printf("Predicted Label: %d\n", pipeline.getPredictedClassLabel());
	}

	//constructor
	Livetest()
	{
		printf("Initializing...\n");
		sub1 = nh.subscribe("/featureVectorX", 100, &Livetest::callBack, this);  //Don't use "/featureVector"! it's been used somewhere! triggering weird bug!
		printf("1\n");
		pub = nh.advertise<std_msgs::String>("finalLabel", 100);   //publish to the analyzer that directly publishes to interpreter
		printf("2\n");
		receivedFrameNum = 0;

		LabelledTimeSeriesClassificationData trainingData;
		    
		if( !trainingData.load("/home/radar/catkin_ws/src/beginner_tutorials/src/train.grt") ){
			cout << "ERROR: Failed to load training data from file\n";
		}
	
		HMM hmm;
		hmm.enableNullRejection(true);

		pipeline.setClassifier( hmm );
		
		if (pipeline.train(trainingData)) printf("pipeline training successful\n");
		else printf("Failed to train pipeline\n");
		
		if (pipeline.getTrained()) printf("Pipeline trained!\n");
		else printf("Pipeline not trained...\n");
        	printf("Done\n");
	}
};


int main(int argc, char **argv)
{
    	//subscribe to featureVector
	ros::init(argc, argv, "live_test");
	Livetest obj;
	ros::spin();
    	return 0;
  
}
