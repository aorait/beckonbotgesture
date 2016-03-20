#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>                                                                                                                                                           
#include <fstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <ctime>
#include <sys/time.h> 
#include <iomanip>

using namespace std;

//timer related: timer controls file dump!
//if (first_receive): mark the starting timer
//if (this receive time - last receive time > 500ms : dump first, then continue to receive
struct timeval t1, t2;
double elapsedTime;

int classID = 0; //THIS IS TO CHANGE WITH EVERY CLASS! (the only argument)
const int dimension=4;
double normVal;
int round_next_step;  //this is to enforce an order of execution of the 4 callbacks, i.e. each callback should only be called once every round!
int total_lines;  //count how many lines are there in a GRT data sample
std::ofstream out;  //the GRT file, write incrememtally
std::mutex g_mutex;
std::vector<std::vector<double>> data_sample;
bool first_bag = true;
int global_counter = 0;

class FormFeatureVector
{
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub1,sub2,sub3,sub4,sub5,sub6,sub7;
	std_msgs::Float64MultiArray featureVec;
	double x_vals[dimension], y_vals[dimension], z_vals[dimension];
	//double dist[dimension*dimension];
public:

	void callBack1(const geometry_msgs::PoseStamped msg)
	{
    /*
        printf("in 1\n");
        g_mutex.lock();
        while (round_next_step != 1)
        {
            g_mutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            g_mutex.lock();
        }
      */ 
       // printf("callBack 1\n");
       
        round_next_step ++;
  		x_vals[0]= msg.pose.position.x;
  		y_vals[0]= msg.pose.position.y;
  		z_vals[0]= msg.pose.position.z;
       // g_mutex.unlock();

	}
	void callBack2(const geometry_msgs::PoseStamped msg)
	{
    /*
        printf("in 2\n");
        g_mutex.lock();
        while (round_next_step != 2)
        {
            g_mutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            g_mutex.lock();
        }
        */
       
       // printf("callBack 2\n");
       
        round_next_step ++;
  		x_vals[1]= msg.pose.position.x;
  		y_vals[1]= msg.pose.position.y;
  		z_vals[1]= msg.pose.position.z;
    //    g_mutex.unlock();
  	}
  	
  	void callBack3(const geometry_msgs::PoseStamped msg)
	{
    /*
        printf("in 3\n");
        g_mutex.lock();
        while (round_next_step != 3)
        {
            g_mutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            g_mutex.lock();
        }
        */
       
       // printf("callBack 3\n");
        round_next_step ++;
  		x_vals[2]= msg.pose.position.x;
  		y_vals[2]= msg.pose.position.y;
  		z_vals[2]= msg.pose.position.z;
      //  g_mutex.unlock();
  	}

  	void callBack4(const geometry_msgs::PoseStamped msg)
	{
   /*
        printf("in 4\n");
        g_mutex.lock();
        while (round_next_step != 4)
        {
            g_mutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            g_mutex.lock();
        }
     */ 
      //  printf("callBack 4\n");
       
        round_next_step = 1;   //This is the last call of this round
  		x_vals[3]= msg.pose.position.x;
  		y_vals[3]= msg.pose.position.y;
  		z_vals[3]= msg.pose.position.z;
  		featureVec.data.clear();
  		calcDist();
  	//	pub.publish(featureVec);//just for human to see
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
        if (first_bag)
        {
            first_bag = false;
            data_sample.push_back(featureVec.data);
        }
        else if (elapsedTime > 500)  //it's been more than 400 ms since last received a round of positions
        {
            dump_to_file();
            global_counter ++;
            printf("Dump file: %d\n", global_counter);
            data_sample.push_back(featureVec.data);
        }
        else
        {
            data_sample.push_back(featureVec.data);
        }
        gettimeofday(&t1, NULL);
       // g_mutex.unlock();
  	}

  	void calcDist()
    {
  		double dist;
  		for(int i=0;i<dimension;i++)
        {
  			for(int j=i+1;j<dimension;j++)
            {
  				dist=sqrt(pow((x_vals[i]-x_vals[j]),2)+pow((y_vals[i]-y_vals[j]),2)+pow((z_vals[i]-z_vals[j]),2));
  				dist=dist/normVal;
  				featureVec.data.push_back(dist);
  			}
  		}
  	}

    void dump_to_file()  //dump a GRT data sample to the GRT file
    {
//        printf("DUMPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP!\n");
        out << "************TIME_SERIES************" << endl;
        out << "ClassID: " << classID << endl;
        out << "TimeSeriesLength: " << data_sample.size() << endl;
        out << "TimeSeriesData: " << endl;
        int dumpsize = data_sample.size();
	
	if (data_sample.size() < 200)
		printf("WARNING: data sample size is %d\n", data_sample.size());
	

        for (int i=0; i<dumpsize; i++)
        {
            std::vector<double> line = data_sample[i];
            int linelength = line.size();
	    int cnt = 0; //for debug
            for (int j=0; j<linelength; j++)
            {
                //out << std::fixed << std::setprecision(7) << line[j] << "\t";
		if (cnt != 0 && /*cnt != 3 &&*/ cnt != 5)   //skip "Shoulder <-> UpperArm and LowerArm <-> Hand"
			out << (int)(line[j]*500) << "\t";
		//for debug
		cnt++;
	    }
            out << endl;
        }
        data_sample.clear();
    }

//constructor
	FormFeatureVector()
    {
        printf("Initialization...");
		sub1 = nh.subscribe("/Shoulder/pose", 100, &FormFeatureVector::callBack1, this);
		sub2 = nh.subscribe("/UpperArm/pose", 100, &FormFeatureVector::callBack2, this);
		sub3 = nh.subscribe("/LowerArm/pose", 100, &FormFeatureVector::callBack3, this);
		sub4 = nh.subscribe("/Hand/pose", 100, &FormFeatureVector::callBack4, this);
       // sub5 = nh.subscribe("/dump_command", 100, &FormFeatureVector::dump_to_file, this);
		pub = nh.advertise<std_msgs::Float64MultiArray>("featureVectorT", 100);   //for human ovservation only
        printf("\n");
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
//  cout<<"enter hand to hand distance in T pose"<<endl;
//  cin>>normVal;
  //I will assume that normalization has been done...
    classID = atoi(argv[1]);
    printf("classID: %d\n", classID);
    gettimeofday(&t1, NULL);
    normVal = 1.0;
    round_next_step = 1;
    total_lines = 0;
    out.open("/home/radar/catkin_ws/src/beginner_tutorials/src/train.grt", std::ios::app); //append-write
    data_sample.clear();
    ros::init(argc, argv, "featureNode");
    FormFeatureVector obj;
    ros::spin();
    return 0;
}
