#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class reach_goal{
    private:
        ros::NodeHandle n;
        ros::Subscriber reach_goal_state;
        bool condition_state = false;
    public:
        bt::Condition condition_reach_goal;
        reach_goal();
        void conditionSet();
        void stateCallback(const std_msgs::String::ConstPtr &msg);
};

reach_goal :: reach_goal() : condition_reach_goal("arrive goal"){
    reach_goal_state = n.subscribe("/reach_goal_state", 1,  &reach_goal::stateCallback, this);
}

void reach_goal :: conditionSet(){
    condition_reach_goal.set(condition_state);
    condition_reach_goal.publish();
    return;
}

void reach_goal :: stateCallback(const std_msgs::String::ConstPtr &msg){
    if(msg->data == "reached"){
        ROS_INFO("reached");
        condition_state = true;
    }else{
        ROS_INFO("reaching");
    }
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "state_reach_goal");
    reach_goal state_reach_goal;
    while(ros::ok()){
        state_reach_goal.conditionSet();
        ros::spinOnce();
    }
    return 0;
}