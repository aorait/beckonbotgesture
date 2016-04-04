#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include <list>
#include "GRT.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <sys/time.h>

#define dimension 4
#define delta_threshold 10
#define window_size 5

using namespace GRT;

std::ofstream out;
FILE *fin = NULL;
FILE *fout = NULL;

class MyFrame
{
public:
	int vecData[dimension];

	MyFrame(int array[dimension])
	{
		for (int i=0; i<dimension; i++)  //the "normalization" is here
		{
			vecData[i] = array[i];
		}
	}
};

class Filter
{
	ros::NodeHandle nh;
	ros::Publisher pub;  //publish to topic "finalLabel"
	ros::Subscriber sub1;  //listen to topic "featureVector"
	TimeSeriesClassificationData testingData;
	MatrixDouble testingSample;
	int receivedFrameNum;  //let's keep buffer size, sample at granularity of 5, start at receiving buffer size
	std::list<MyFrame> buffer;
	HMM hmm;

	
	std::vector<MyFrame> recording_frames;
	std::vector<MyFrame> last_few_frames;
	int frame_cnt;
	bool recording;  //means now recording 
	struct timeval t1, t2;
	//suggestion: focus on detecting "starting point", let it auto stop after 2.5s
public:

	int cal_delta(MyFrame f1, MyFrame f2)
	{
		int sum = 0;
		for (int i=0; i<dimension; i++)
		{
			int diff = f1.vecData[i] - f2.vecData[i];
			sum += diff>0? diff : 0-diff;
		}
		return sum;
	}

	void run(void)
	{
        int array[dimension];
        int frame_count = 0;  //in practice, this is that "gettimeofday" to count 2.5 seconds, here it counts 260 frames
        printf("Alive2.5\n");
        printf("%ld\n", fin);
        while (fscanf(fin, "%d%d%d%d", &array[0], &array[1], &array[2], &array[3]) != EOF) 
        {
              //  printf("%d %d %d %d\n", array[0], array[1], array[2], array[3]);
                receivedFrameNum ++;
                if (!recording)
                {
                    if (last_few_frames.size() >= window_size)
                        last_few_frames.erase(last_few_frames.begin());
                    MyFrame frame(array);
                    last_few_frames.push_back(frame);
                    if (last_few_frames.size() >= window_size)
                    {
                        int delta = cal_delta(last_few_frames.at(0), last_few_frames.at(window_size - 1));
                        printf("Delta: %d\n", delta);
                        if (delta > delta_threshold)
                        {
                            recording = true;
                            frame_count = 0;
                            recording_frames.clear();
                        }
                    }
                }
                if (recording)
                {
                    MyFrame frame(array);
                    recording_frames.push_back(frame);
                    frame_count ++; 
                    if (frame_count > 260)
                    {
                        //dump a recording
                        printf("Dump\n");
                        int len = recording_frames.size();
                        fprintf(fout, "***********************\n");
                        for (int i=0; i<len; i++)
                        {
                            for (int j=0; j<dimension; j++)
                            {
                                fprintf(fout, "%d\t", (int)recording_frames.at(i).vecData[j]);
                            }
                            fprintf(fout,"\n");
                        }
                        recording = false;
                        last_few_frames.clear();
                    }
                    
                }
        }
        fclose(fin);
        fclose(fout);

	}

	//constructor
	Filter()
	{
		receivedFrameNum = 0;
		frame_cnt = 0;
		recording = false;
		last_few_frames.clear();
        fin = fopen("/home/radar/catkin_ws/src/beginner_tutorials/src/filter_test.in", "r");
    //    printf("Alive2\n");
        run();   //simunate the continuous stream, replacing the callback function
	}
};



int main(int argc, char **argv)
{
    	//subscribe to featureVector
	ros::init(argc, argv, "filter_test");
	fout=fopen("/home/radar/catkin_ws/src/beginner_tutorials/src/filter_test.out","w");
	Filter obj;
    printf("Alive1\n");
  	return 0;
  
}
