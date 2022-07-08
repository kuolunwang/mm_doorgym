#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class init{
    private:
        ros::NodeHandle n;
        ros::Subscriber init_state;
        bool condition_state = false;
    public:
        bt::Condition condition_init;
        init();
        void conditionSet();
        void stateCallback(const std_msgs::String::ConstPtr &msg);
};

init :: init() : condition_init("init finish"){
    init_state = n.subscribe("/init_state", 1,  &init::stateCallback, this);
}

void init :: conditionSet(){
    condition_init.set(condition_state);
    condition_init.publish();
    return;
}

void init :: stateCallback(const std_msgs::String::ConstPtr &msg){
    if(msg->data == "init_finished"){
        ROS_INFO("inited");
        condition_state = true;
    }else{
        ROS_INFO("initing");
    }
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "state_init");
    init state_init;
    while(ros::ok()){
        state_init.conditionSet();
        ros::spinOnce();
    }
    return 0;
}