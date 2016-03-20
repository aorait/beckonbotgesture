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
	HMM hmm;
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
		if (receivedFrameNum<buffer_size)
		{
			MyFrame vec(array);  //constructor
			buffer.push_back(vec);
			return;
		}

		//sample frequency control
		if (receivedFrameNum % 5 != 0)
		{
			MyFrame vec(array);
			buffer.push_back(vec);
			buffer.pop_front();  //this WILL destroy the poped object, no need to delete
			return;
		}
		//otherwise, push, do the classification, pop
		MyFrame vec(array);
		buffer.push_back(vec);
		testingData.clear();
		testingData.setNumDimensions( dimension );
		testingData.setDatasetName("TestingData");
		testingSample.clear();
		VectorDouble sample( dimension );
		for (std::list<MyFrame>::iterator it=buffer.begin(); it != buffer.end(); ++it)
		{
			MyFrame frame = *it;
			for (int j=0; j<dimension; j++)
			{
				sample[j] = frame.vecData[j];
			}
			testingSample.push_back( sample );
		}
		testingData.addSample( 0 , testingSample );   //the label should not matter here...
		buffer.pop_front();
		//classification
		hmm.predict( testingData[0].getData() );
		double likelyhood = hmm.getMaximumLikelihood();
		if (likelyhood > likelyhood_threshold)
		{
			cout << " PredictedClassLabel: " << hmm.getPredictedClassLabel() << endl;
			cout << " MaxLikelihood: " << hmm.getMaximumLikelihood() << endl;
		}
		//Publish this label to topic "finalLabel", to do in future
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
		//Load the HMM model from a file
    		if( !hmm.load( "HMMModel.grt" ) )  //dir of the cpp source files...
		{
        		cout << "ERROR: Failed to load the model from a file!\n";
    		}
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
