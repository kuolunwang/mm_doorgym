#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "ros/service_client.h"
#include <std_srvs/Trigger.h>
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class open_action{
    private:
        ros::NodeHandle n;
        ros::ServiceClient open_srv;
    public:
        bt::Action action_open;
        open_action();
        void actionSet(int state);
        void cmdopening();
};

open_action :: open_action() : action_open("open door"){
    open_srv = n.serviceClient<std_srvs::Trigger>("/door_open");
}

void open_action :: actionSet(int state){
    switch(state){
        case 1:
            action_open.set_success();
            break;
        case 0:
            action_open.set_running();
            break;
        case -1:
            action_open.set_failure();
            break;
    }
    action_open.publish();
    return;
}

void open_action :: cmdopening(){
    std_srvs::Trigger srv;
    if(open_srv.call(srv)){ return; }
    else{ ROS_INFO("Error"); }
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "action_open");
    open_action in_ac;
    while(ros::ok()){
        if(in_ac.action_open.is_active() && in_ac.action_open.active_has_changed()){
            ROS_INFO("Action: open activiate");
        }
        if(in_ac.action_open.active_has_changed() && !(in_ac.action_open.is_active())){
            ROS_INFO("Action: Done");
            in_ac.actionSet(1);
        }
        if(in_ac.action_open.is_active()){
            in_ac.actionSet(0);
            in_ac.cmdopening();
        }
        ros::spinOnce();
    }
    return 0;
}