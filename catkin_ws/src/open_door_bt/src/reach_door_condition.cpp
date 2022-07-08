#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class reach_door{
    private:
        ros::NodeHandle n;
        ros::Subscriber reach_door_state;
        bool condition_state = false;
    public:
        bt::Condition condition_reach_door;
        reach_door();
        void conditionSet();
        void stateCallback(const std_msgs::String::ConstPtr &msg);
};

reach_door :: reach_door() : condition_reach_door("arrive door"){
    reach_door_state = n.subscribe("/reach_door_state", 1,  &reach_door::stateCallback, this);
}

void reach_door :: conditionSet(){
    condition_reach_door.set(condition_state);
    condition_reach_door.publish();
    return;
}

void reach_door :: stateCallback(const std_msgs::String::ConstPtr &msg){
    if(msg->data == "reached"){
        ROS_INFO("reached");
        condition_state = true;
    }else{
        ROS_INFO("reaching");
    }
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "state_reach_door");
    reach_door state_reach_door;
    while(ros::ok()){
        state_reach_door.conditionSet();
        ros::spinOnce();
    }
    return 0;
}