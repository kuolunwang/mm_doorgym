#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class door{
    private:
        ros::NodeHandle n;
        ros::Subscriber door_state;
        bool condition_state = false;
    public:
        bt::Condition condition_door;
        door();
        void conditionSet();
        void stateCallback(const std_msgs::String::ConstPtr &msg);
};

door :: door() : condition_door("door open enough"){
    door_state = n.subscribe("/door_state", 1,  &door::stateCallback, this);
}

void door :: conditionSet(){
    condition_door.set(condition_state);
    condition_door.publish();
    return;
}

void door :: stateCallback(const std_msgs::String::ConstPtr &msg){
    if(msg->data == "opened"){
        ROS_INFO("opened");
        condition_state = true;
    }else{
        ROS_INFO("opening");
    }
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "state_door");
    door state_door;
    while(ros::ok()){
        state_door.conditionSet();
        ros::spinOnce();
    }
    return 0;
}