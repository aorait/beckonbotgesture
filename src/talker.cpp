/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>

#include "GRT.h"
using namespace GRT;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

//Load the training data
    TimeSeriesClassificationData trainingData;
    
     if( !trainingData.load("/home/radar/catkin_ws/src/beginner_tutorials/src/train.grt") ){
    // if( !trainingData.load("/home/radar/beckonbot/opti_raw.grt") ){
    //   if( !trainingData.load("/home/radar/beckonbot/HMMTrainingData.grt") ){
        cout << "ERROR: Failed to load training data from file\n";
        return EXIT_FAILURE;
    }
    
    //Remove 10% of the training data to use as test data
    TimeSeriesClassificationData testData = trainingData.partition( 90 );
    
    //Create a new HMM instance
    HMM hmm;
    
    //Set the HMM as a Continuous HMM
    hmm.setHMMType( HMM_CONTINUOUS );
    
    //Set the downsample factor, a higher downsample factor will speed up the prediction time, but might reduce the classification accuracy
    //hmm.setDownsampleFactor( 5 );
    //hmm.setDownsampleFactor( 10 );   strange: not only is it faster, but it is also more accurate......
    //hmm.setDownsampleFactor( 15 );   //strange: not only is it faster, but it is also more accurate......
    hmm.setDownsampleFactor( 30 );
    
    //Set the committee size, this sets the (top) number of models that will be used to make a prediction
    hmm.setCommitteeSize( 10 );
    
    //Tell the hmm algorithm that we want it to estimate sigma from the training data
    hmm.setAutoEstimateSigma( true );
//hmm.setAutoEstimateSigma( false );
    
    //Set the minimum value for sigma, you might need to adjust this based on the range of your data
    //If you set setAutoEstimateSigma to false, then all sigma values will use the value below
    hmm.setSigma( 20.0 );
//hmm.setSigma( 1000.0 );
    
    //Set the HMM model type to LEFTRIGHT with a delta of 1, this means the HMM can only move from the left-most state to the right-most state
    //in steps of 1
    hmm.setModelType( HMM_LEFTRIGHT );
    hmm.setDelta( 1 );
    
    //Train the HMM model
    if( !hmm.train( trainingData ) ){
        cout << "ERROR: Failed to train the HMM model!\n";
        return false;
    }
    
    //Save the HMM model to a file
    if( !hmm.save( "HMMModel.grt" ) ){
        cout << "ERROR: Failed to save the model to a file!\n";
        return false;
    }
    
    //Load the HMM model from a file
    if( !hmm.load( "HMMModel.grt" ) ){
        cout << "ERROR: Failed to load the model from a file!\n";
        return false;
    }

    //Compute the accuracy of the HMM models using the test data
    double numCorrect = 0;
    double numTests = 0;
    for(UINT i=0; i<testData.getNumSamples(); i++){
        
        UINT classLabel = testData[i].getClassLabel();
        hmm.predict( testData[i].getData() );
        
        if( classLabel == hmm.getPredictedClassLabel() ) numCorrect++;
        numTests++;
        
        VectorDouble classLikelihoods = hmm.getClassLikelihoods();
        VectorDouble classDistances = hmm.getClassDistances();
        
        cout << "ClassLabel: " << classLabel;
        cout << " PredictedClassLabel: " << hmm.getPredictedClassLabel();
        cout << " MaxLikelihood: " << hmm.getMaximumLikelihood();
        
        cout << "  ClassLikelihoods: ";
        for(UINT k=0; k<classLikelihoods.size(); k++){
            cout << classLikelihoods[k] << "\t";
        }
        
        cout << "ClassDistances: ";
        for(UINT k=0; k<classDistances.size(); k++){
            cout << classDistances[k] << "\t";
        }
        cout << endl;
    }
    
    cout << "Test Accuracy: " << numCorrect/numTests*100.0 << endl;
    
    return true;










  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%
/*Calling ros::spinOnce() here is not necessary for this simple program, because we are not receiving any callbacks. However, if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called. So, add it for good measure.*/

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }

  return 0;
}
// %EndTag(FULLTEXT)%
