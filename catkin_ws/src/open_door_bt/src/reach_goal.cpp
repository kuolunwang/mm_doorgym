#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/service_client.h"
#include <std_srvs/Trigger.h>
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class reach_goal_action{
    private:
        ros::NodeHandle n;
        ros::ServiceClient goal_srv;
    public:
        bt::Action action_reach_goal;
        reach_goal_action();
        void actionSet(int state);
        void checkgoal();
};

reach_goal_action :: reach_goal_action() : action_reach_goal("nav to goal"){
    goal_srv = n.serviceClient<std_srvs::Trigger>("/check_goal");
}

void reach_goal_action :: actionSet(int state){
    switch(state){
        case 1:
            action_reach_goal.set_success();
            break;
        case 0:
            action_reach_goal.set_running();
            break;
        case -1:
            action_reach_goal.set_failure();
            break;
    }
    action_reach_goal.publish();
    return;
}

void reach_goal_action :: checkgoal(){
    std_srvs::Trigger srv;
    if(goal_srv.call(srv)){ return; }
    else{ ROS_INFO("Error"); }
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "action_reach_goal");
    reach_goal_action in_ac;
    while(ros::ok()){
        if(in_ac.action_reach_goal.is_active() && in_ac.action_reach_goal.active_has_changed()){
            ROS_INFO("Action: open activiate");
        }
        if(in_ac.action_reach_goal.active_has_changed() && !(in_ac.action_reach_goal.is_active())){
            ROS_INFO("Action: Done");
            in_ac.actionSet(1);
        }
        if(in_ac.action_reach_goal.is_active()){
            in_ac.actionSet(0);
            in_ac.checkgoal();
        }
        ros::spinOnce();
    }
    return 0;
}