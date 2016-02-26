#include "ros/ros.h"
#include "std_msgs/Float64.h"

class myClass {
public:
    int data; 
    

    myClass(){}
    myClass(float d){
	data=d;
	}
    void incData(){
	data++;
	}
    void decData(){
	data--;
	}
};
